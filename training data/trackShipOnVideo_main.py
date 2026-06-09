#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ship tracking with YOLO on multiple video files selected from a monitoring log.
"""

import os
import sys
import cv2
import csv
from ultralytics import YOLO
from tqdm import tqdm
from pathlib import Path

# ─── USER SETTINGS ──────────────────────────────────────────────────────────────
LOG_FILE_PATH    = r"E:/[Drone_recreational_ship_RNL_project]/watching_boat_passing_hydrophone/monitoring log.txt"  # path to monitoring log
VIDEO_ROOT_PATH  = r"E:/[Drone_recreational_ship_RNL_project]/watching_boat_passing_hydrophone"  # top folder of videos
EXPORT_FOLDER    = r"E:/[Drone_recreational_ship_RNL_project]"  # where outputs go
WEIGHTS_PATH     = r"E:/[Drone_recreational_ship_RNL_project]/shipTrack_yoloV11m.onnx"
START_INDEX      = 2    # 1-based index of video in the list to start processing
FRAME_INTERVAL   = 5    # process every Nth frame
TARGET_SIZE      = (640, 640)
PADDING_COLOR    = (114, 114, 114)
CONF_THRESH      = 0.25
IOU_THRESH       = 0.45
# ──────────────────────────────────────────────────────────────────────────────────

def parse_log_file(path):
    """
    Read the monitoring log and return a set of base filenames
    with exactly one boat tag (no commas).
    """
    single = set()
    with open(path, 'r', encoding='utf-8') as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('['):
                continue
            parts = line.split(None, 1)
            if len(parts) != 2:
                continue
            base, tags = parts
            if ',' not in tags:
                single.add(base)
    return single


def find_videos(base_names, root):
    """
    Walk through root, collecting MP4s whose stem is in base_names.
    Returns a sorted list of full paths.
    """
    matches = []
    for root_dir, _, files in os.walk(root):
        for fname in files:
            if fname.lower().endswith('.mp4'):
                stem = os.path.splitext(fname)[0]
                if stem in base_names:
                    matches.append(os.path.join(root_dir, fname))
    return sorted(matches)


def letterbox(img, target_size, color=PADDING_COLOR):
    """Resize and pad to target_size, keeping aspect ratio, and return padding offsets."""
    h, w = img.shape[:2]
    tw, th = target_size
    scale = min(tw / w, th / h)
    nw, nh = int(w * scale), int(h * scale)
    dw, dh = tw - nw, th - nh
    top, bottom = dh // 2, dh - (dh // 2)
    left, right = dw // 2, dw - (dw // 2)
    resized = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    padded = cv2.copyMakeBorder(resized, top, bottom, left, right,
                                 cv2.BORDER_CONSTANT, value=color)
    return padded, scale, left, top


def track_video(video_path, model, idx, total):
    """Process a single video: detect and track ships, save video & CSV."""
    print(f"\n[{idx}/{total}] Processing: {video_path}")
    cap = cv2.VideoCapture(video_path)
    if not cap.isOpened():
        print(f"[ERROR] Cannot open {video_path}")
        return

    fps = cap.get(cv2.CAP_PROP_FPS) or 30.0
    total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")

    base = os.path.splitext(os.path.basename(video_path))[0]
    out_vid = os.path.join(EXPORT_FOLDER, f"{base}_tracked.mp4")
    out_csv = os.path.join(EXPORT_FOLDER, f"{base}_tracked.csv")

    out = cv2.VideoWriter(out_vid, fourcc, fps, TARGET_SIZE)
    track_pts = []
    csv_rows = []

    for frame_idx in tqdm(range(total_frames), desc=f"Frames ({base})"):
        ret, frame = cap.read()
        if not ret:
            break
        img, scale, left, top = letterbox(frame, TARGET_SIZE)

        # Only run detection every FRAME_INTERVAL frames
        if frame_idx % FRAME_INTERVAL == 0:
            result = model(img, conf=CONF_THRESH, iou=IOU_THRESH)[0]
            best_box, best_area = None, 0
            for box in result.boxes:
                x1, y1, x2, y2 = map(int, box.xyxy.tolist()[0])
                area = (x2 - x1) * (y2 - y1)
                if area > best_area:
                    best_area = area
                    best_box = (x1, y1, x2, y2,
                                float(box.conf[0]), int(box.cls[0]))

            if best_box:
                x1, y1, x2, y2, conf, cls = best_box
                cx_padded = (x1 + x2) // 2
                cy_padded = (y1 + y2) // 2
                cx_orig = (cx_padded - left) / scale
                cy_orig = (cy_padded - top) / scale
                track_pts.append((cx_padded, cy_padded))
                csv_rows.append([
                    "track01",
                    frame_idx + 1,
                    cx_orig,
                    cy_orig,
                    "",
                    "",
                    "without calf",
                    "surface"
                ])
                label = model.names[cls]
                cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(img, f"{label} {conf:.2f}",
                            (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX,
                            0.5, (0, 255, 0), 1)

        # Draw track lines
        for i in range(1, len(track_pts)):
            cv2.line(img, track_pts[i - 1], track_pts[i], (0, 0, 255), 2)

        out.write(img)

    # Write CSV
    with open(out_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            "track", "frame", "x", "y",
            "latitude", "longitude",
            "calf_status", "marker_status"
        ])
        writer.writerows(csv_rows)

    cap.release()
    out.release()
    print(f"✅ Done: {out_vid}\n   CSV: {out_csv}")


def main():
    # Validate paths
    if not Path(LOG_FILE_PATH).is_file():
        print(f"[ERROR] Log not found: {LOG_FILE_PATH}")
        sys.exit(1)
    if not Path(VIDEO_ROOT_PATH).is_dir():
        print(f"[ERROR] Videos root not found: {VIDEO_ROOT_PATH}")
        sys.exit(1)
    if not Path(WEIGHTS_PATH).is_file():
        print(f"[ERROR] Weights not found: {WEIGHTS_PATH}")
        sys.exit(1)
    os.makedirs(EXPORT_FOLDER, exist_ok=True)

    # Gather videos
    bases = parse_log_file(LOG_FILE_PATH)
    vids = find_videos(bases, VIDEO_ROOT_PATH)
    total = len(vids)
    if total == 0:
        print("No matching videos found.")
        return

    print(f"Found {total} video(s). Starting from index {START_INDEX}.")
    model = YOLO(WEIGHTS_PATH)

    for idx, vpath in enumerate(vids, start=1):
        if idx < START_INDEX:
            continue
        track_video(vpath, model, idx, total)

if __name__ == "__main__":
    main()

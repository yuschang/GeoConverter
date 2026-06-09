import subprocess

YOLO12_DIR   = r"D:/software_dev/dolphin_track/yolov12"
WEIGHTS_PATH = r"D:/software_dev/dolphin_track/shipDetect_YoloV12m_250629.pt"
VIDEO_PATH   = r"E:/[Drone_recreational_ship_RNL_project]/watching_boat_passing_hydrophone/DJI_202506231601_002/DJI_20250623160227_0001_V.MP4"
OUTPUT_PATH  = r"D:/software_dev/output_video.mp4"

cmd = [
    "python",
    f"{YOLO12_DIR}\\detect.py",
    "--weights", WEIGHTS_PATH,
    "--source", VIDEO_PATH,
    "--imgsz", "640",
    "--conf", "0.25",
    "--iou", "0.45",
    "--save-vid",
    "--save-path", OUTPUT_PATH
]

subprocess.run(cmd, check=True)
print(f"Finished inference, video saved to {OUTPUT_PATH}")
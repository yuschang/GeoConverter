# 🐬 GeoConverter (Dolphin Track)

**A PyQt5-based video analysis tool for tracking marine animals from drone footage**

*Developed by Dr. Changsoo Kim*

GeoConverter is a desktop application designed for researchers studying marine mammals (primarily dolphins) from aerial drone video. It provides frame-by-frame video navigation, manual point tracking, geolocation conversion using drone SRT metadata, and real-time distance measurement — all within a single, integrated interface.

> **Current Version:** 1.6.3v_Dist Meas

---

## Table of Contents

1. [Features](#features)
2. [Requirements](#requirements)
3. [Getting Started](#getting-started)
4. [User Interface Overview](#user-interface-overview)
5. [Detailed Usage Guide](#detailed-usage-guide)
6. [File Formats](#file-formats)
7. [Camera Parameters](#camera-parameters)
8. [Troubleshooting](#troubleshooting)
9. [Citation](#citation)


---

## Features

- **Video playback & navigation** — Load MP4 videos and step through frames with configurable intervals. Supports folder-based batch loading for processing multiple video files in sequence.
- **Manual point tracking** — Place markers on individual animals across frames to build movement tracks. Each track can be categorized (e.g., "with calf," "Dolphin Pod," etc.) and each marker can be tagged as "underwater" or "surface."
- **Geolocation conversion** — Convert pixel-coordinate tracks to real-world latitude/longitude using drone SRT metadata and camera intrinsic parameters, powered by the `drone_geolocator` module.
- **Distance measurement** — Measure the real-world distance (in meters) between any two points on a single frame by clicking directly on the video.
- **Geo-referenced plotting** — Visualize converted tracks on a local metric coordinate plot (Easting vs. Northing) within the application.
- **CSV import/export** — Save and reload tracks with full geolocation data, marker status, and category labels for downstream analysis.
- **Zoom & pan** — Zoom in/out and pan across high-resolution frames for precise marker placement.
- **SRT overlay** — Toggle an on-screen display of the drone's telemetry metadata (GPS, altitude, gimbal angles, etc.) for any frame.

---

## Requirements

### Python Version

- Python 3.8 or higher

### Dependencies

Install all required packages via pip:

```bash
pip install PyQt5 opencv-python-headless pandas numpy matplotlib
```

### Additional Module

This application depends on a companion module, **`drone_geolocator`**, which handles SRT file parsing and the pixel-to-geolocation conversion. Ensure that `drone_geolocator.py` is placed in the same directory as the main script, or is installed as a package accessible on your Python path.

The `drone_geolocator` module must expose the following functions:

- `parse_srt_file(srt_path)` — Parses a DJI-style `.SRT` subtitle file and returns a dictionary keyed by frame number, where each value is a dictionary of metadata fields (latitude, longitude, altitude, gimbal angles, etc.).
- `convert_tracks_to_geo(tracks_df, srt_data, ...)` — Accepts a DataFrame of pixel-coordinate tracks and SRT metadata, then returns a DataFrame with added `latitude` and `longitude` columns.

---

## Getting Started

Here is a minimal walkthrough to get you tracking dolphins in under five minutes.

### Step 1 — Load a video

Click **Load Video** to open a single `.mp4` file, or click **Load Folder** to load an entire directory of `.mp4` files for sequential processing.

### Step 2 — Navigate frames

Use the frame slider, the **Go** button (jump to a specific frame number), or the arrow keys (`A`/`D` or `←`/`→`) to move through the video. The **Step** field controls how many frames the Play button advances per tick.

### Step 3 — Create a track

Click the **Marker Mode** button. A dialog will ask you to select the dolphin's status category (e.g., "without calf," "with Neonate"). Once confirmed, a new track is created and the cursor changes to a crosshair. Click on the video to place markers on each frame where the animal appears.

### Step 4 — Save your work

Click **Save Tracks to CSV**. The application automatically computes geolocation for every marker and writes a CSV file alongside the video (same filename, `.csv` extension).

### Step 5 — Visualize

Click **plot GeoLocation** to see all tracks plotted in local metric coordinates on the built-in chart.

---

## User Interface Overview

The interface is divided into two main panels:

### Left Panel — Video & Navigation

| Control | Description |
|---|---|
| **Load Video** | Open a single video file |
| **Load Folder** | Open a folder of `.mp4` files for batch processing |
| **< / >** | Navigate to the previous or next video in the folder |
| **Play / Pause** | Start or stop automatic playback |
| **Step** | Number of frames to advance per playback tick |
| **Show SRT Info** | Toggle the on-screen telemetry overlay |
| **Zoom In / Out / Reset** | Adjust the view magnification |
| **Distance Measurement** | Toggle two-point distance measurement mode |
| **FocalLen, PixPitch, pp_x, pp_y** | Camera intrinsic parameters (see [Camera Parameters](#camera-parameters)) |
| **Frame slider** | Drag to scrub through the video |
| **Go** | Jump to a specific frame number |
| **◀ / ▶ with Jump field** | Step backward or forward by a custom number of frames |

### Right Panel — Tracking & Analysis

| Control | Description |
|---|---|
| **Marker Mode** | Toggle marker placement on/off; creates a new track when activated |
| **Only Selected Track** | When checked, only the currently selected track is drawn on the video |
| **Hide Tracks** | Hide all track overlays from the video display |
| **Tracks list** | Shows all tracks with their category labels; click to select and jump to the track |
| **Category dropdown** | Change the category of the selected track |
| **Remove Track** | Delete the selected track and all its markers |
| **Marker Status** | Set the selected marker to "underwater" or "surface" |
| **Marker Summary** | Lists all markers for the current track with frame and coordinate info |
| **Delete Marker** | Remove the currently selected marker |
| **Save Tracks to CSV** | Export all tracks with geolocation to a CSV file |
| **Marine / Terrestrial** | Select the target environment for altitude reference in geolocation |
| **plot GeoLocation** | Compute geolocation and plot all tracks on the embedded chart |

---

## Detailed Usage Guide

### Video Navigation

- **Keyboard shortcuts:** Press `A` or `←` to go back, and `D`, `→`, or `S` to go forward. The number of frames jumped is determined by the **Jump** input field.
- **Frame stepping during playback:** The **Step** field sets how many frames are skipped between each playback tick. Set it to `1` for smooth playback or to a larger value (e.g., `30`) to quickly scan through footage.

### Creating and Managing Tracks

1. Click **Marker Mode** to begin a new track. A dialog prompts you to choose a category.
2. Click on the animal in the video frame to place a marker. Each click adds a point to the current track at the current frame.
3. Navigate to the next frame and click again to continue the track.
4. Click **Marker Mode** again (uncheck it) to stop adding markers.
5. To resume adding markers to an existing track, select the track in the list, then re-enable Marker Mode. Note: enabling Marker Mode always creates a *new* track.

**Track categories** describe the animal's social context or identity:

| Category | Description |
|---|---|
| without calf | Adult dolphin, no calf present |
| with Neonate | Adult accompanied by a neonate |
| with calf | Adult accompanied by a calf |
| with Juvenile | Adult accompanied by a juvenile |
| Dolphin Pod | Group of dolphins tracked as a unit |
| Watching boat | Animal observed watching a boat |
| SB, MB, LB, FB, JB, YY | Individual ID codes |
| Car, Box | Non-animal reference objects |

**Marker status** can be set to either `underwater` or `surface` for each individual marker point, reflecting the animal's position at that moment.

### Distance Measurement

1. Click the **Distance Measurement** button (it turns green when active).
2. Click a first point on the video — a green square marker appears.
3. Click a second point — a red square marker appears, a dashed yellow line connects them, and the real-world distance in meters is displayed at the midpoint.
4. Click again to start a new measurement (resets the previous one).
5. Click the button again to exit distance measurement mode.

Distance is computed by converting both pixel coordinates to geographic positions via the drone's SRT metadata and camera parameters, then calculating the Haversine distance between them.

> **Note:** Distance measurement requires a valid `.SRT` file loaded alongside the video and correct camera parameters.

### Geolocation Conversion

The application converts pixel coordinates to real-world latitude/longitude using:

- **Drone telemetry** from the `.SRT` file (GPS position, altitude, gimbal pitch/roll/yaw)
- **Camera intrinsic parameters** entered in the UI (focal length, pixel pitch, principal point)

**Environment setting:**

- **Marine** — Assumes the target is at sea level (elevation = 0). Use this for dolphins and other marine animals.
- **Terrestrial** — Uses the drone's relative height above takeoff to estimate target elevation. Use this for land-based subjects.

### SRT Overlay

Check **Show SRT Info** to display the drone's telemetry data directly on the video frame. This is useful for verifying GPS coordinates, altitude, and gimbal angles at any point during the flight.

---

## File Formats

### Input Files

| File | Description |
|---|---|
| `.mp4` | Drone video file |
| `.SRT` | DJI-style subtitle file containing per-frame telemetry. Must share the same base filename as the video (e.g., `flight01.mp4` and `flight01.SRT`) |
| `.csv` | Previously saved track data (auto-loaded when opening a video if the CSV exists) |

### Output Files

| File | Description |
|---|---|
| `.csv` | Track data with columns: `track`, `frame`, `x`, `y`, `latitude`, `longitude`, `calf_status`, `marker_status` |

The CSV is saved with the same base filename as the video. For example, opening `flight01.mp4` and saving will produce `flight01.csv`.

### CSV Column Descriptions

| Column | Type | Description |
|---|---|---|
| `track` | string | Track identifier (e.g., `track01`, `track02`) |
| `frame` | integer | Frame number where the marker was placed |
| `x` | float | Horizontal pixel coordinate in the video frame |
| `y` | float | Vertical pixel coordinate in the video frame |
| `latitude` | float | Computed geographic latitude (WGS84), or `NaN` if conversion failed |
| `longitude` | float | Computed geographic longitude (WGS84), or `NaN` if conversion failed |
| `calf_status` | string | Track category label |
| `marker_status` | string | Either `underwater` or `surface` |

---

## Camera Parameters

The four camera intrinsic parameters must be set correctly for accurate geolocation. These values depend on the specific drone and camera model used.

| Parameter | UI Field | Default | Description |
|---|---|---|---|
| Focal length | FocalLen | `0.0098` m | True focal length of the camera lens in meters |
| Pixel pitch | PixPitch | `0.0000033` m | Physical size of a single sensor pixel in meters |
| Principal point X | pp_x | `2007` px | Horizontal coordinate of the optical center in pixels |
| Principal point Y | pp_y | `1131` px | Vertical coordinate of the optical center in pixels |

The default values are configured for a typical DJI drone camera sensor. If you are using a different camera, consult your camera's specification sheet or perform a camera calibration to obtain accurate values.

---

## Troubleshooting

**Video does not load**
Ensure the file is a valid video format readable by OpenCV. MP4 (H.264) is recommended. Some codec issues can be resolved by installing `opencv-python` instead of `opencv-python-headless`.

**SRT data not loading**
The `.SRT` file must be in the same directory as the video and share the same base filename. The extension must be uppercase `.SRT`. Verify the file follows the DJI SRT format expected by `drone_geolocator.parse_srt_file()`.

**Geolocation returns NaN**
This can happen if the SRT metadata is missing or malformed for the frame in question, or if the camera parameters are incorrect. Check that your SRT file contains valid GPS, altitude, and gimbal data for the frames you are tracking.

**Distance measurement shows "N/A"**
Same causes as NaN geolocation above. Ensure SRT data is loaded and camera parameters are set before using distance measurement.

**Application crashes on startup**
Verify all dependencies are installed. In particular, ensure PyQt5 is compatible with your Python version and operating system.

---

## Citation

If you use GeoConverter in your research, please cite the associated publication:

```
[Citation details to be added upon publication]
```

---


## Acknowledgments
This research was supported by the Basic Science Research Program of the Research Institute for Basic Sciences (RIBS) of Jeju National University through the National Research Foundation of Korea (NRF), funded by the Ministry of Education. (2019R1A6A1A10072987). This research was also supported by the Korea Institute of Marine Science and Technology Promotion (KIMST), funded by the Ministry of Oceans and Fisheries (RS−2023−00256122). 

GeoConverter was developed to support field research on marine mammal behavior from drone-based aerial surveys. We thank all contributors and field researchers who provided feedback during development.

For questions, bug reports, or feature requests, please open an issue on this repository.

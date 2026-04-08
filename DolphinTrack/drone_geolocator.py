import os
import re
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from math import sin, cos, atan2, sqrt, radians, degrees
from scipy.spatial.transform import Rotation as R

# --- SRT PARSER ---
def parse_srt_file(srt_path):
    srt_data = {}
    with open(srt_path, encoding='utf-8') as f:
        srt_text = f.read()
    blocks = re.split(r'\n\s*\n', srt_text)
    for block in blocks:
        frame_match = re.search(r'FrameCnt:\s*(\d+)', block)
        if not frame_match:
            continue
        frame = int(frame_match.group(1))
        meta = {}
        pairs = re.findall(r'\[([^\]:]+):\s*([^\]]+)\]', block)
        for k, v in pairs:
            k = k.strip().lower()
            try:
                v = float(v)
            except ValueError:
                v = v.strip()
            meta[k] = v
        alt_match = re.search(r'rel_alt:\s*([-\d.]+)\s*abs_alt:\s*([-\d.]+)', block)
        if alt_match:
            meta['rel_alt'] = float(alt_match.group(1))
            meta['abs_alt'] = float(alt_match.group(2))
        att_match = re.search(r'gb_yaw:\s*([-\d.]+)\s*gb_pitch:\s*([-\d.]+)\s*gb_roll:\s*([-\d.]+)', block)
        if att_match:
            meta['gb_yaw'] = float(att_match.group(1))
            meta['gb_pitch'] = float(att_match.group(2))
            meta['gb_roll'] = float(att_match.group(3))
        srt_data[frame] = meta
    return srt_data

# --- COORDINATE CONVERSION FUNCTIONS ---
a = 6378137.0
f = 1 / 298.257223563
e2 = f * (2 - f)
b = a * (1 - f)

def geodetic_to_ecef(lat_deg, lon_deg, h):
    lat = radians(lat_deg)
    lon = radians(lon_deg)
    N = a / sqrt(1 - e2 * sin(lat)**2)
    x = (N + h) * cos(lat) * cos(lon)
    y = (N + h) * cos(lat) * sin(lon)
    z = (N * (1 - e2) + h) * sin(lat)
    return np.array([x, y, z])

def ecef_to_geodetic_fixed_height(x, y, z, target_h, tol=1e-8, max_iter=10):
    p = sqrt(x**2 + y**2)
    lat = atan2(z, p)
    for _ in range(max_iter):
        N = a / sqrt(1 - e2 * sin(lat) ** 2)
        lat_new = atan2(z, p * (1 - e2 * N / (N + target_h)))
        if abs(lat_new - lat) < tol:
            lat = lat_new
            break
        lat = lat_new
    lon = atan2(y, x)
    return degrees(lat), degrees(lon), target_h

def eul2rotm(eul, seq='ZYX'):
    r = R.from_euler(seq, eul)
    return r.as_matrix()

def geo_locate_target(pixel_u, pixel_v, pixel_pitch, focal_len, pp_x, pp_y, R_cam2ned,
                      drone_lat, drone_lon, drone_alt, target_h):
    drone_ecef = geodetic_to_ecef(drone_lat, drone_lon, drone_alt)
    x_sensor = (pixel_v - pp_y) * pixel_pitch
    y_sensor = (pp_x - pixel_u) * pixel_pitch
    los_cam = np.array([x_sensor, y_sensor, -focal_len])
    los_cam /= np.linalg.norm(los_cam)
    los_ned = R_cam2ned @ los_cam
    lat0 = radians(drone_lat)
    lon0 = radians(drone_lon)
    cosLat = cos(lat0)
    sinLat = sin(lat0)
    cosLon = cos(lon0)
    sinLon = sin(lon0)
    R_ecef2ned = np.array([
        [-sinLat*cosLon, -sinLat*sinLon, cosLat],
        [-sinLon,         cosLon,        0],
        [-cosLat*cosLon, -cosLat*sinLon, -sinLat]
    ])
    los_ecef = R_ecef2ned.T @ los_ned
    los_ecef /= np.linalg.norm(los_ecef)
    a_sph = a + target_h
    b_sph = a_sph * sqrt(1 - e2)
    A = (los_ecef[0]/a_sph)**2 + (los_ecef[1]/a_sph)**2 + (los_ecef[2]/b_sph)**2
    B = 2 * (drone_ecef[0]*los_ecef[0]/a_sph**2 +
             drone_ecef[1]*los_ecef[1]/a_sph**2 +
             drone_ecef[2]*los_ecef[2]/b_sph**2)
    C = (drone_ecef[0]/a_sph)**2 + (drone_ecef[1]/a_sph)**2 + (drone_ecef[2]/b_sph)**2 - 1
    D = B**2 - 4*A*C
    if D < 0:
        return np.nan, np.nan, np.nan
    t1 = (-B + sqrt(D)) / (2*A)
    t2 = (-B - sqrt(D)) / (2*A)
    t = t2 if t2 > 0 and t2 < t1 else t1
    intersect = drone_ecef + t * los_ecef
    lat, lon, h = ecef_to_geodetic_fixed_height(intersect[0], intersect[1], intersect[2], target_h)
    return lat, lon, h

def convert_tracks_to_geo(
    tracks, srt_data,
    true_focal_len=12.3e-3,
    pixel_pitch=3.3e-6,
    img_w=3840,
    img_h=2160,
    pp_x=None,
    pp_y=None,
    use_relative_height=False
):
    """
    :param tracks: DataFrame of tracked points (must include columns x, y, frame, etc.)
    :param srt_data: Parsed metadata dict from parse_srt_file()
    :param true_focal_len: Effective focal length in meters
    :param pixel_pitch: Sensor pixel pitch in meters
    :param img_w: Image width (pixels)
    :param img_h: Image height (pixels)
    :param pp_x: Principal point x in pixels (default is img_w/2)
    :param pp_y: Principal point y in pixels (default is img_h/2)
    :param use_relative_height: if True, use (abs_alt - rel_alt) for ground; if False, use 0
    """
    if pp_x is None:
        pp_x = img_w / 2
    if pp_y is None:
        pp_y = img_h / 2
    lats, lons = [], []
    for idx, row in tracks.iterrows():
        frame = int(row['frame'])
        meta = srt_data.get(frame)
        if not meta:
            lats.append(np.nan)
            lons.append(np.nan)
            continue
        dzoom = meta.get('dzoom_ratio', 1.0)
        focal_len = true_focal_len * dzoom
        yaw = radians(meta.get('gb_yaw', 0))
        pitch = radians(meta.get('gb_pitch', 0) + 90)
        roll = radians(meta.get('gb_roll', 0))
        rot = eul2rotm([yaw, pitch, roll], 'ZYX')
        droneLat = meta.get('latitude')
        droneLon = meta.get('longitude')
        abs_alt = meta.get('abs_alt')
        rel_alt = meta.get('rel_alt')
        if use_relative_height and abs_alt is not None and rel_alt is not None:
            tgt_h = abs_alt - rel_alt
        else:
            tgt_h = 0
        u = row['x']
        v = row['y']
        lat, lon, h = geo_locate_target(
            u, v, pixel_pitch, focal_len, pp_x, pp_y, rot,
            droneLat, droneLon, abs_alt, tgt_h
        )
        lats.append(lat)
        lons.append(lon)
    tracks = tracks.copy()
    tracks['latitude'] = lats
    tracks['longitude'] = lons
    return tracks

def plot_tracks(tracks, show=True, save_path=None):
    plt.figure(figsize=(8, 6))
    for name, grp in tracks.groupby('track'):
        plt.plot(grp['longitude'], grp['latitude'], marker='o', label=name)
    plt.title('Tracks (Geographic Coordinates)')
    plt.xlabel('Longitude')
    plt.ylabel('Latitude')
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.xlim([1.265665515283309*1e2, 1.265668439324230*1e2])
    plt.ylim([33.453235626855665, 33.453366466628196])
    if save_path:
        plt.savefig(save_path)
    if show:
        plt.show()

# Example function to run the full pipeline with parameter control
def run_gui_conversion_and_plot(
    true_focal_len=9.6e-3,
    pixel_pitch=3.3e-6,
    img_w=3840,
    img_h=2160,
    pp_x=None,
    pp_y=None,
    use_relative_height=False
):
    try:
        from easygui import fileopenbox
    except ImportError:
        print("easygui is not installed.")
        return

    csv_path = fileopenbox("Select the CSV file", filetypes=["*.csv"])
    if not csv_path: return
    base_name = os.path.splitext(csv_path)[0]
    for ext in ['.SRT', '.srt']:
        srt_path = base_name + ext
        if os.path.exists(srt_path):
            break
    else:
        srt_path = fileopenbox("Select the matching SRT file", filetypes=["*.srt"])
        if not srt_path: return

    for enc in ['utf-8', 'cp949', 'euc-kr', 'latin1']:
        try:
            tracks = pd.read_csv(csv_path, encoding=enc)
            print(f"Loaded CSV using encoding: {enc}")
            break
        except UnicodeDecodeError:
            continue
    else:
        raise RuntimeError("Could not decode CSV. Try saving it as UTF-8.")

    srt_data = parse_srt_file(srt_path)
    tracks = convert_tracks_to_geo(
        tracks, srt_data,
        true_focal_len=true_focal_len,
        pixel_pitch=pixel_pitch,
        img_w=img_w,
        img_h=img_h,
        pp_x=pp_x,
        pp_y=pp_y,
        use_relative_height=use_relative_height
    )
    plot_tracks(tracks)
    return tracks

if __name__ == '__main__':
    # Example: pass pp_x and pp_y directly here if needed
    run_gui_conversion_and_plot(
        true_focal_len=9.6e-3, 
        pixel_pitch=3.3e-6, 
        img_w=3840, 
        img_h=2160, 
        pp_x=2007,   # Example: NOT center, but set by user
        pp_y=1132,
        use_relative_height=True
    )

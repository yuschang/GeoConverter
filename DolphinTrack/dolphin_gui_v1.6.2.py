# -*- coding: utf-8 -*-
"""
Created on Thu Jul 10 12:02:48 2025

@author: yustc
"""
import sys
import cv2
import os
import re
import csv
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QVBoxLayout,
    QHBoxLayout, QFileDialog, QSlider, QLineEdit, QCheckBox,
    QListWidget, QToolButton, QGraphicsView, QGraphicsScene,
    QGraphicsPixmapItem, QComboBox, QTableWidget, QTableWidgetItem, QDialog
)
from PyQt5.QtGui import QPixmap, QImage, QPainter, QPen, QColor, QCursor, QIntValidator, QFont, QBrush
from PyQt5.QtCore import Qt, QTimer, QPointF, QEvent, QRectF
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
import pandas as pd
import drone_geolocator
import numpy as np
import math

class VideoViewer(QMainWindow):
    TRACK_CATEGORIES = [
        "without calf", "with Neonate", "with calf", "with Juvenile",
        "Dolphin Pod","Watching boat", "SB", "MB", "LB", "FB", "JB", "YY", "Car", "Box"
    ]

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Dolphin Track (1.6.3v_Dist Meas) _ Dr. Changsoo Kim")
        self.setGeometry(200, 100, 1200, 800)
        # Video folder management
        self.video_folder = ""
        self.video_list = []
        self.video_index = 0
        # Core state
        self.video_path = ""
        self.cap = None
        self.total_frames = 0
        self.current_frame = 0
        self.zoom_applied = False
        self.vid_width = 0
        self.vid_height = 0
        self.timer = QTimer(); self.timer.timeout.connect(self.next_frame)
        self.mark_mode = False
        self.current_track = ""
        self.track_count = 0
        self.track_points = {}
        self.track_status = {}
        self.marker_statuses = {}
        self.toggle_only_selected = False
        self.hide_tracks = False
        self.srt_data = {}
        self.zoom_factor = 1.0
        self.last_pan_point = None
        self.targetEnv = 'm'

        # Distance measurement state
        self.distance_mode = False
        self.distance_point1 = None  # (x, y) in scene coords
        self.distance_point2 = None  # (x, y) in scene coords
        self.measured_distance = None  # distance in meters

        self.initUI()

    def initUI(self):
        int_validator = QIntValidator(1, 1000000, self)
        # Primary controls: load/play
        self.load_btn = QPushButton("Load Video"); self.load_btn.clicked.connect(self.load_video)
        self.play_btn = QPushButton("Play"); self.play_btn.clicked.connect(self.toggle_play)
        self.interval_input = QLineEdit("1"); self.interval_input.setFixedWidth(50); self.interval_input.setValidator(int_validator)
        self.overlay_srt_checkbox = QCheckBox("Show SRT Info"); self.overlay_srt_checkbox.stateChanged.connect(self.handle_overlay_toggle)
        ctrl_layout1 = QHBoxLayout(); ctrl_layout1.setSpacing(3); ctrl_layout1.setContentsMargins(0,0,0,0)
        ctrl_layout1.addWidget(self.load_btn); ctrl_layout1.addWidget(self.play_btn)
        ctrl_layout1.addWidget(self.make_label("Step:",30)); ctrl_layout1.addWidget(self.interval_input)
        ctrl_layout1.addWidget(self.overlay_srt_checkbox)
        # Folder controls: load folder, nth video, prev/next
        self.load_folder_btn = QPushButton("Load Folder"); self.load_folder_btn.clicked.connect(self.load_folder)
        self.nth_input = QLineEdit("1"); self.nth_input.setFixedWidth(50); self.nth_input.setValidator(int_validator)
        self.nth_input.setPlaceholderText("Video #"); self.nth_input.editingFinished.connect(self.load_nth_video)
        self.prev_video_btn = QPushButton("<"); self.prev_video_btn.clicked.connect(self.prev_video)
        self.next_video_btn = QPushButton(">"); self.next_video_btn.clicked.connect(self.next_video)
        folder_layout = QHBoxLayout(); folder_layout.setSpacing(3); folder_layout.setContentsMargins(0,0,0,0)
        folder_layout.addWidget(self.load_folder_btn); folder_layout.addWidget(self.nth_input)
        folder_layout.addWidget(self.prev_video_btn); folder_layout.addWidget(self.next_video_btn)
        # Zoom & Save + Distance Measurement
        self.zoom_in_btn = QPushButton("Zoom in"); self.zoom_in_btn.clicked.connect(self.zoom_in)
        self.zoom_out_btn = QPushButton("Zoom out"); self.zoom_out_btn.clicked.connect(self.zoom_out)
        self.zoom_reset_btn = QPushButton("Reset Zoom"); self.zoom_reset_btn.clicked.connect(self.reset_zoom)

        # Distance Measurement toggle button
        self.distance_btn = QPushButton("Distance Measurement")
        self.distance_btn.setCheckable(True)
        self.distance_btn.setStyleSheet("QPushButton { background-color: #cc4444; color: white; font-weight: bold; }"
                                        "QPushButton:checked { background-color: #44aa44; color: white; font-weight: bold; }")
        self.distance_btn.toggled.connect(self.toggle_distance_mode)

        ctrl_layout2 = QHBoxLayout(); ctrl_layout2.setSpacing(3); ctrl_layout2.setContentsMargins(0,0,0,0)
        ctrl_layout2.addWidget(self.zoom_in_btn); ctrl_layout2.addWidget(self.zoom_out_btn)
        ctrl_layout2.addWidget(self.zoom_reset_btn); ctrl_layout2.addWidget(self.distance_btn)
        # Camera params
        self.focal_len_input = QLineEdit("0.0098"); self.focal_len_input.setFixedWidth(60)
        self.pixel_pitch_input = QLineEdit("0.0000033"); self.pixel_pitch_input.setFixedWidth(60)
        self.pp_x_input = QLineEdit("2007"); self.pp_x_input.setFixedWidth(60)
        self.pp_y_input = QLineEdit("1131"); self.pp_y_input.setFixedWidth(60)
        ctrl_layout3 = QHBoxLayout(); ctrl_layout3.setSpacing(3); ctrl_layout3.setContentsMargins(0,0,0,0)
        ctrl_layout3.addWidget(self.make_label("FocalLen:",60)); ctrl_layout3.addWidget(self.focal_len_input)
        ctrl_layout3.addWidget(self.make_label("PixPitch:",60)); ctrl_layout3.addWidget(self.pixel_pitch_input)
        ctrl_layout3.addWidget(self.make_label("pp_x:",40)); ctrl_layout3.addWidget(self.pp_x_input)
        ctrl_layout3.addWidget(self.make_label("pp_y:",40)); ctrl_layout3.addWidget(self.pp_y_input)
        ctrl_layout3.setAlignment(Qt.AlignLeft)

        # Left panel assembly
        left = QVBoxLayout(); left.addLayout(ctrl_layout1); left.addLayout(folder_layout)
        left.addLayout(ctrl_layout2); left.addLayout(ctrl_layout3)
        # Video display
        self.scene = QGraphicsScene(); self.graphicsView = QGraphicsView(self.scene)
        self.graphicsView.setFixedSize(1024,640); self.graphicsView.setDragMode(QGraphicsView.ScrollHandDrag)
        self.graphicsView.viewport().installEventFilter(self); self.pixmap_item = QGraphicsPixmapItem(); self.scene.addItem(self.pixmap_item)
        left.addWidget(self.graphicsView)
        # Frame slider & nav
        self.path_text = QLineEdit(); self.path_text.setReadOnly(True); left.addWidget(self.path_text)
        self.slider = QSlider(Qt.Horizontal); self.slider.setMinimum(1); self.slider.valueChanged.connect(self.slider_changed); left.addWidget(self.slider)
        self.current_frame_display = QLineEdit("0"); self.current_frame_display.setReadOnly(True); self.current_frame_display.setValidator(int_validator)
        self.goto_input = QLineEdit(); self.goto_input.setPlaceholderText("Frame #"); self.goto_input.setValidator(int_validator)
        self.goto_btn = QPushButton("Go"); self.goto_btn.clicked.connect(self.goto_frame)
        self.backward_btn = QPushButton("\u25C0"); self.backward_btn.clicked.connect(self.jump_backward)
        self.jump_input = QLineEdit(); self.jump_input.setPlaceholderText("Jump"); self.jump_input.setValidator(int_validator)
        self.forward_btn = QPushButton("\u25B6"); self.forward_btn.clicked.connect(self.jump_forward)
        nav = QHBoxLayout(); nav.addWidget(QLabel("Current:")); nav.addWidget(self.current_frame_display)
        nav.addWidget(self.goto_input); nav.addWidget(self.goto_btn)
        nav.addWidget(self.backward_btn); nav.addWidget(self.jump_input); nav.addWidget(self.forward_btn)
        left.addLayout(nav)
        # Right panel: tracks & markers
        self.marker_mode_btn = QToolButton(); self.marker_mode_btn.setText("Marker Mode"); self.marker_mode_btn.setCheckable(True)
        self.marker_mode_btn.clicked.connect(self.toggle_marker_mode)
        self.marker_mode_btn.setStyleSheet("QToolButton{background-color:lightgray;} QToolButton:checked{background-color:lightgreen;}")
        self.toggle_tracks_btn = QCheckBox("Only Selected Track"); self.toggle_tracks_btn.stateChanged.connect(self.toggle_only)
        self.hide_tracks_btn = QPushButton("Hide Tracks"); self.hide_tracks_btn.setCheckable(True); self.hide_tracks_btn.toggled.connect(self.toggle_hide_tracks)
        self.track_list = QListWidget(); self.track_list.itemClicked.connect(self.goto_track_start)
        self.track_category_label = QLabel("Category:"); self.track_category_combo = QComboBox(); self.track_category_combo.addItems(self.TRACK_CATEGORIES)
        self.track_category_combo.currentIndexChanged.connect(self.track_category_changed)
        self.remove_track_btn = QPushButton("Remove Track"); self.remove_track_btn.clicked.connect(self.remove_selected_track)
        self.save_csv_btn = QPushButton("Save Tracks to CSV"); self.save_csv_btn.clicked.connect(self.save_tracks_csv)


        # SRT metadata table
        track_ctrl = QVBoxLayout(); track_ctrl.addWidget(self.toggle_tracks_btn); track_ctrl.addWidget(self.hide_tracks_btn)
        track_ctrl.addWidget(QLabel("Tracks:")); track_ctrl.addWidget(self.track_list)
        track_ctrl.addWidget(self.track_category_label); track_ctrl.addWidget(self.track_category_combo)
        track_ctrl.addWidget(self.remove_track_btn);
        
        self.status_combo = QComboBox(); self.status_combo.addItems(['underwater','surface']); self.status_combo.currentIndexChanged.connect(self.status_combo_changed)
        self.marker_summary = QListWidget(); self.marker_summary.itemSelectionChanged.connect(self.marker_selection_changed)
        self.delete_marker_btn = QPushButton("Delete Marker"); self.delete_marker_btn.clicked.connect(self.delete_selected_marker)
        marker_ctrl = QVBoxLayout(); marker_ctrl.addWidget(QLabel("Marker Status:")); marker_ctrl.addWidget(self.status_combo)
        marker_ctrl.addWidget(QLabel("Marker Summary:")); marker_ctrl.addWidget(self.marker_summary); marker_ctrl.addWidget(self.delete_marker_btn)
        right_top = QHBoxLayout(); right_top.addLayout(track_ctrl,1); right_top.addLayout(marker_ctrl,2)
        # Set up geo plot canvas and figure
        self.geo_figure = Figure(figsize=(4,3))
        self.geo_canvas = FigureCanvas(self.geo_figure)
        self.geoLocation_conversion_btn = QPushButton("plot GeoLocation"); self.geoLocation_conversion_btn.clicked.connect(self.convert_geolocation)
        self.targetHeighRef = QComboBox(); self.targetHeighRef.addItems(['Marine','Terrestrial']); self.targetHeighRef.currentIndexChanged.connect(self.targetEnv_combo_changed)
        geo_ctrl = QVBoxLayout(); geo_ctrl.addWidget(self.targetHeighRef); geo_ctrl.addWidget(self.geoLocation_conversion_btn); geo_ctrl.addWidget(self.geo_canvas)
        right = QVBoxLayout(); right.addWidget(self.marker_mode_btn); right.addLayout(right_top); right.addWidget(self.save_csv_btn)
        right.addLayout(geo_ctrl)
        # Main
        main = QHBoxLayout(); main.addLayout(left,3); main.addLayout(right,1)
        container = QWidget(); container.setLayout(main); self.setCentralWidget(container)


    # ========================
    # Distance Measurement
    # ========================
    def toggle_distance_mode(self, checked):
        """Toggle distance measurement mode on/off."""
        self.distance_mode = checked
        if checked:
            # Entering distance mode: clear previous measurement
            self.distance_point1 = None
            self.distance_point2 = None
            self.measured_distance = None
            # Disable marker mode if active
            if self.mark_mode:
                self.marker_mode_btn.setChecked(False)
                self.mark_mode = False
        else:
            # Exiting distance mode: clear overlay
            self.distance_point1 = None
            self.distance_point2 = None
            self.measured_distance = None
        self.show_frame(self.current_frame)

    def compute_distance_between_points(self, px1, py1, px2, py2):
        """
        Given two pixel coordinates on the current frame, convert each to
        geolocation using the same algorithm as convert_tracks_to_geo, then
        compute the distance in meters between the two geolocations.
        Returns distance in meters, or None if conversion fails.
        """
        if not self.srt_data:
            return None

        # Build a small DataFrame with the two points on the current frame
        data = [
            {'track': '_dist_pt1', 'frame': self.current_frame, 'x': px1, 'y': py1},
            {'track': '_dist_pt2', 'frame': self.current_frame, 'x': px2, 'y': py2},
        ]
        tracks_df = pd.DataFrame(data)

        try:
            true_focal_len = float(self.focal_len_input.text())
            pixel_pitch = float(self.pixel_pitch_input.text())
            pp_x = float(self.pp_x_input.text())
            pp_y = float(self.pp_y_input.text())
        except Exception:
            return None

        if self.targetEnv == 'm':
            use_relative_height = False
        elif self.targetEnv == 't':
            use_relative_height = True
        else:
            use_relative_height = False

        try:
            geo_tracks = drone_geolocator.convert_tracks_to_geo(
                tracks_df,
                self.srt_data,
                true_focal_len=true_focal_len,
                pixel_pitch=pixel_pitch,
                img_w=self.vid_width,
                img_h=self.vid_height,
                pp_x=pp_x,
                pp_y=pp_y,
                use_relative_height=use_relative_height
            )
        except Exception as e:
            print(f"Geolocation conversion error: {e}")
            return None

        if geo_tracks is None or geo_tracks.empty:
            return None

        pt1_row = geo_tracks[geo_tracks['track'] == '_dist_pt1']
        pt2_row = geo_tracks[geo_tracks['track'] == '_dist_pt2']

        if pt1_row.empty or pt2_row.empty:
            return None

        lat1 = pt1_row.iloc[0].get('latitude', np.nan)
        lon1 = pt1_row.iloc[0].get('longitude', np.nan)
        lat2 = pt2_row.iloc[0].get('latitude', np.nan)
        lon2 = pt2_row.iloc[0].get('longitude', np.nan)

        if np.isnan(lat1) or np.isnan(lon1) or np.isnan(lat2) or np.isnan(lon2):
            return None

        # Haversine distance in meters
        R = 6378137.0  # Earth radius in meters
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = (math.sin(dlat / 2) ** 2 +
             math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
             math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        distance = R * c
        return distance

    def handle_distance_click(self, scene_pos):
        """Handle a click in distance measurement mode."""
        x, y = scene_pos.x(), scene_pos.y()

        if self.distance_point1 is None:
            # First click: set point 1
            self.distance_point1 = (x, y)
            self.distance_point2 = None
            self.measured_distance = None
            self.show_frame(self.current_frame)
        elif self.distance_point2 is None:
            # Second click: set point 2 and compute distance
            self.distance_point2 = (x, y)
            dist = self.compute_distance_between_points(
                self.distance_point1[0], self.distance_point1[1],
                x, y
            )
            self.measured_distance = dist
            self.show_frame(self.current_frame)
        else:
            # Both points already set: reset and start over with new point 1
            self.distance_point1 = (x, y)
            self.distance_point2 = None
            self.measured_distance = None
            self.show_frame(self.current_frame)

    def draw_distance_overlay(self, painter):
        """Draw the distance measurement markers, dashed line, and label on the frame."""
        marker_size = 14  # half-side of the square marker

        # Draw point 1 as green square
        if self.distance_point1 is not None:
            x1, y1 = self.distance_point1
            painter.setPen(QPen(QColor(0, 200, 0), 3))
            painter.setBrush(QColor(0, 200, 0, 120))
            painter.drawRect(int(x1 - marker_size), int(y1 - marker_size),
                             marker_size * 2, marker_size * 2)

        # Draw point 2 as red square
        if self.distance_point2 is not None:
            x2, y2 = self.distance_point2
            painter.setPen(QPen(QColor(220, 0, 0), 3))
            painter.setBrush(QColor(220, 0, 0, 120))
            painter.drawRect(int(x2 - marker_size), int(y2 - marker_size),
                             marker_size * 2, marker_size * 2)

        # Draw dashed line between the two points
        if self.distance_point1 is not None and self.distance_point2 is not None:
            x1, y1 = self.distance_point1
            x2, y2 = self.distance_point2

            dash_pen = QPen(QColor(255, 255, 0), 3, Qt.DashLine)
            painter.setPen(dash_pen)
            painter.setBrush(Qt.NoBrush)
            painter.drawLine(QPointF(x1, y1), QPointF(x2, y2))

            # Draw distance label near the midpoint of the line
            mid_x = (x1 + x2) / 2
            mid_y = (y1 + y2) / 2

            if self.measured_distance is not None:
                dist_text = f"{self.measured_distance:.2f} m"
            else:
                dist_text = "N/A"

            # Background rectangle for readability
            font = QFont()
            font.setPointSize(28)
            font.setBold(True)
            painter.setFont(font)

            text_rect = painter.fontMetrics().boundingRect(dist_text)
            padding = 8
            bg_rect = QRectF(mid_x + 10, mid_y - text_rect.height() - padding,
                             text_rect.width() + padding * 2, text_rect.height() + padding * 2)
            painter.setPen(Qt.NoPen)
            painter.setBrush(QColor(0, 0, 0, 160))
            painter.drawRoundedRect(bg_rect, 6, 6)

            # Draw the text
            painter.setPen(QPen(QColor(255, 255, 0)))
            painter.drawText(int(mid_x + 10 + padding), int(mid_y), dist_text)


    def load_folder(self):
        folder = QFileDialog.getExistingDirectory(self, "Select Video Folder")
        if not folder: return
        self.video_folder = folder
        files = sorted([f for f in os.listdir(folder) if f.lower().endswith('.mp4')])
        if not files: return
        self.video_list = files
        self.video_index = 0
        self.nth_input.setText('1')
        self.open_video_file(os.path.join(folder, files[0]))
        

    def load_nth_video(self):
        if not self.video_folder or not self.video_list: return
        try:
            idx = int(self.nth_input.text()) - 1
        except:
            return
        if 0 <= idx < len(self.video_list):
            self.video_index = idx
            self.open_video_file(os.path.join(self.video_folder, self.video_list[idx]))

    def load_nth_video(self):
           if not self.video_folder or not self.video_list: return
           try:
               idx = int(self.nth_input.text()) - 1
           except:
               return
           if 0 <= idx < len(self.video_list):
               self.video_index = idx
               self.open_video_file(os.path.join(self.video_folder, self.video_list[idx]))

    def prev_video(self):
        if not self.video_list: return
        self.video_index = max(0, self.video_index - 1)
        self.nth_input.setText(str(self.video_index+1))
        self.open_video_file(os.path.join(self.video_folder, self.video_list[self.video_index]))

    def next_video(self):
        if not self.video_list: return
        self.video_index = min(len(self.video_list)-1, self.video_index + 1)
        self.nth_input.setText(str(self.video_index+1))
        self.open_video_file(os.path.join(self.video_folder, self.video_list[self.video_index]))

    def open_video_file(self, path):
        self.cap = cv2.VideoCapture(path)
        if not self.cap.isOpened(): return
        self.video_path = path
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.vid_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.vid_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.path_text.setText(path)
        self.slider.setMaximum(self.total_frames)
        self.load_srt_file(path)
        self.load_tracks_csv(path)
        self.show_frame(1)
        self.reset_zoom()
        self.update_marker_summary()
        self.update_track_list_labels()

            
    def track_category_changed(self, index):
        if not self.current_track: return
        self.track_status[self.current_track] = self.TRACK_CATEGORIES[index]
        self.update_track_list_labels()
          
        
    def on_track_selected(self, item):
        track = item.text().split(' [',1)[0]
        self.current_track = track
        # Load category
        cat = self.track_status.get(track, self.TRACK_CATEGORIES[0])
        idx = self.TRACK_CATEGORIES.index(cat)
        self.track_status_combo.blockSignals(True)
        self.track_status_combo.setCurrentIndex(idx)
        self.track_status_combo.blockSignals(False)
        self.update_marker_summary()

    def track_status_changed(self, idx):
        if not self.current_track: return
        new_cat = self.TRACK_CATEGORIES[idx]
        self.track_status[self.current_track] = new_cat
        self.update_track_list_labels()

    def update_track_list_labels(self):
        self.track_list.blockSignals(True)
        self.track_list.clear()
        for track in sorted(self.track_points.keys()):
            cat = self.track_status.get(track, self.TRACK_CATEGORIES[0])
            self.track_list.addItem(f"{track} [{cat}]")
        if self.current_track:
            matches = self.track_list.findItems(self.current_track, Qt.MatchStartsWith)
            if matches: self.track_list.setCurrentItem(matches[0])
        self.track_list.blockSignals(False)
        
        
    def make_label(self, text, width):
        label = QLabel(text)
        label.setFixedWidth(width)
        return label

    def toggle_hide_tracks(self, checked):
        self.hide_tracks = checked
        if self.hide_tracks:
            self.hide_tracks_btn.setText("Show Tracks")
        else:
            self.hide_tracks_btn.setText("Hide Tracks")
        self.show_frame(self.current_frame)

    def zoom_in(self): 
        self.zoom_factor *= 1.25
        self.graphicsView.scale(1.25, 1.25)
        self.zoom_applied = True

    def zoom_out(self): 
        self.zoom_factor *= 0.8
        self.graphicsView.scale(0.8, 0.8)
        self.zoom_applied = True

    def reset_zoom(self):
        self.graphicsView.resetTransform()
        self.zoom_factor = 1.0
        self.graphicsView.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def eventFilter(self, obj, event):
        if obj is self.graphicsView.viewport():
            # Set cursor based on active mode
            if self.distance_mode:
                if self.graphicsView.viewport().cursor().shape() != Qt.CrossCursor:
                    self.graphicsView.viewport().setCursor(QCursor(Qt.CrossCursor))
            elif self.mark_mode:
                if self.graphicsView.viewport().cursor().shape() != Qt.CrossCursor:
                    self.graphicsView.viewport().setCursor(QCursor(Qt.CrossCursor))
            else:
                if self.graphicsView.viewport().cursor().shape() != Qt.ArrowCursor:
                    self.graphicsView.viewport().setCursor(QCursor(Qt.ArrowCursor))

            if event.type() == QEvent.MouseButtonPress:
                # Distance measurement mode takes priority
                if self.distance_mode:
                    pos = self.graphicsView.mapToScene(event.pos())
                    self.handle_distance_click(pos)
                    return True
                elif self.mark_mode:
                    pos = self.graphicsView.mapToScene(event.pos())
                    pts = self.track_points.setdefault(self.current_track, [])
                    idx = len(pts)
                    pts.append((self.current_frame, (pos.x(), pos.y())))
                    self.marker_statuses[(self.current_track, self.current_frame, idx)] = 'underwater'
                    self.show_frame(self.current_frame)
                    self.update_marker_summary(keep_row=idx)
                    self.marker_summary.setCurrentRow(idx)
                    return True
                elif event.button() == Qt.LeftButton:
                    self.last_pan_point = event.pos()
            elif event.type() == QEvent.MouseMove and self.last_pan_point:
                delta = self.last_pan_point - event.pos()
                self.last_pan_point = event.pos()
                self.graphicsView.horizontalScrollBar().setValue(
                    self.graphicsView.horizontalScrollBar().value() + delta.x())
                self.graphicsView.verticalScrollBar().setValue(
                    self.graphicsView.verticalScrollBar().value() + delta.y())
                return True
            elif event.type() == QEvent.MouseButtonRelease:
                self.last_pan_point = None
        return False

    def keyPressEvent(self, event):
        try:
            v = int(self.jump_input.text())
        except:
            v = 1
        if event.key() == Qt.Key_A or event.key() == Qt.Key_Left:
            self.show_frame(max(1, self.current_frame - v))
        elif event.key() == Qt.Key_D or event.key() == Qt.Key_Right or event.key() == Qt.Key_S:
            self.show_frame(min(self.total_frames, self.current_frame + v))
        else:
            super().keyPressEvent(event)

    def toggle_marker_mode(self):
        if self.marker_mode_btn.isChecked():
            # Disable distance mode if active
            if self.distance_mode:
                self.distance_btn.setChecked(False)
                self.distance_mode = False

            calf_status, ok = self.ask_track_status()
            if not ok:
                self.marker_mode_btn.setChecked(False)
                return
            self.mark_mode = True
            self.track_count += 1
            self.current_track = f"track{self.track_count:02d}"
            self.track_points.setdefault(self.current_track, [])  # Ensure key exists
            track_label = f"{self.current_track} [{calf_status}]"
            self.track_list.addItem(track_label)
            self.track_status[self.current_track] = calf_status
        else:
            self.mark_mode = False
        self.update_marker_summary()
        self.update_track_list_labels()
        
    def ask_track_status(self):
        from PyQt5.QtWidgets import QDialog, QVBoxLayout, QPushButton
        dlg = QDialog(self)
        layout = QVBoxLayout()
        combo = QComboBox()
        combo.addItems([
            "without calf",
            "with Neonate",
            "with calf",
            "with Juvenile",
            "Dolphin Pod",
            "Watching boat",
            "SB",
            "MB",
            "LB",
            "FB",
            "JB",
            "YY",
            "Car",
            "Box"
        ])
        layout.addWidget(QLabel("Select dolphin status for this track:"))
        layout.addWidget(combo)
        ok_btn = QPushButton("OK")
        ok_btn.clicked.connect(dlg.accept)
        layout.addWidget(ok_btn)
        dlg.setLayout(layout)
        result = dlg.exec_()
        return combo.currentText(), result == QDialog.Accepted

    def update_track_list_labels(self):
        self.track_list.blockSignals(True)
        self.track_list.clear()
        for track in sorted(self.track_points.keys()):
            calf_status = self.track_status.get(track, "without calf")
            label = f"{track} [{calf_status}]"
            self.track_list.addItem(label)
        if self.current_track and self.current_track in sorted(self.track_points.keys()):
            idx = list(sorted(self.track_points.keys())).index(self.current_track)
            self.track_list.setCurrentRow(idx)
        self.track_list.blockSignals(False)

    def toggle_only(self, s): 
        self.toggle_only_selected = bool(s)
        self.show_frame(self.current_frame)

    def load_video(self):
        # path, _ = QFileDialog.getOpenFileName(self, "Open Video", "", "*.mp4 *.avi")
        path, _ = QFileDialog.getOpenFileName(self, "Open Video", "", "")

        if not path: return
        self.cap = cv2.VideoCapture(path)
        self.video_path = path
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.path_text.setText(path)
        self.slider.setMaximum(self.total_frames)
        self.load_srt_file(path)
        self.load_tracks_csv(path)
        self.show_frame(1)
        self.reset_zoom()
        self.update_marker_summary()
        self.update_track_list_labels()
        self.vid_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.vid_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        
        
    def save_tracks_csv(self):
        if not self.video_path:
            return
        csv_path = os.path.splitext(self.video_path)[0] + '.csv'
    
        # --- [1] Always re-calculate geolocation before saving ---
        # 1. Prepare DataFrame for tracks
        data = []
        for track, points in self.track_points.items():
            for idx, (frame, (x, y)) in enumerate(points):
                data.append({
                    'track': track,
                    'frame': frame,
                    'x': x,
                    'y': y,
                    'marker_status': self.marker_statuses.get((track, frame, idx), "")
                })
        if not data or not self.srt_data:
            print("No tracking or SRT data available for geolocation.")
            return
        tracks_df = pd.DataFrame(data)
    
        # 2. Get camera parameters from UI
        try:
            true_focal_len = float(self.focal_len_input.text())
            pixel_pitch = float(self.pixel_pitch_input.text())
            pp_x = float(self.pp_x_input.text())
            pp_y = float(self.pp_y_input.text())
        except Exception as e:
            print(f"Error reading parameters: {e}")
            return
    
        # use_relative_height = self.targetEnv == 't'
        
    
        print(self.targetEnv)
        
        if self.targetEnv == 'm':
            use_relative_height = False  # target height = 0
        elif self.targetEnv == 't':
            use_relative_height = True  # target height = abs_h - rel_h
            
            
        print(use_relative_height)
            
        # 3. Geolocate
        geo_tracks = drone_geolocator.convert_tracks_to_geo(
            tracks_df,
            self.srt_data,
            true_focal_len=true_focal_len,
            pixel_pitch=pixel_pitch,
            img_w=self.vid_width,
            img_h=self.vid_height,
            pp_x=pp_x,
            pp_y=pp_y,
            use_relative_height=use_relative_height
        )
        # Save latest geolocated tracks for use elsewhere if desired
        self.last_geolocated_tracks = geo_tracks
    
        # 4. Build lookup: (track, frame, x, y) -> (lat, lon)
        geo_lookup = {}
        if geo_tracks is not None and not geo_tracks.empty:
            for _, row in geo_tracks.iterrows():
                key = (row['track'], int(row['frame']), round(row['x'], 2), round(row['y'], 2))
                geo_lookup[key] = (row.get('latitude', np.nan), row.get('longitude', np.nan))
    
        # 5. Write CSV
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['track', 'frame', 'x', 'y', 'latitude', 'longitude', 'calf_status', 'marker_status'])
            for track, points in self.track_points.items():
                calf_status = self.track_status.get(track, "")
                for idx, (frame, (x, y)) in enumerate(points):
                    mstat = self.marker_statuses.get((track, frame, idx), "underwater")
                    latitude, longitude = np.nan, np.nan
                    key = (track, int(frame), round(x, 2), round(y, 2))
                    if key in geo_lookup:
                        latitude, longitude = geo_lookup[key]
                    writer.writerow([track, frame, x, y, latitude, longitude, calf_status, mstat])
        print(f"Tracks with geolocation saved: {csv_path}")

    def load_tracks_csv(self, video_path):
        self.track_points.clear()
        self.track_status.clear()
        self.marker_statuses.clear()
        self.track_list.clear()
        self.current_track = ""
        csv_path = os.path.splitext(video_path)[0] + '.csv'
        if not os.path.exists(csv_path):
            return
        tracks = {}
        statuses = {}
        marker_statuses = {}
        with open(csv_path, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                track = row['track']
                frame = int(row['frame'])
                x = float(row['x'])
                y = float(row['y'])
                calf_status = row.get('calf_status', 'without calf')
                mstat = row.get('marker_status', 'underwater')
                idx = len(tracks.setdefault(track, []))
                tracks[track].append((frame, (x, y)))
                statuses[track] = calf_status
                marker_statuses[(track, frame, idx)] = mstat
        self.track_points = tracks
        self.track_status = statuses
        self.marker_statuses = marker_statuses
        self.update_track_list_labels()
        if self.track_points:
            self.current_track = sorted(self.track_points.keys())[0]
            self.track_count = len(self.track_points)
        else:
            self.current_track = ""
            self.track_count = 0
        self.update_marker_summary()

              
    def load_srt_file(self, video_path):
        base, _ = os.path.splitext(video_path)
        srt = base + ".SRT"
        self.srt_data.clear()
        if not os.path.exists(srt):
            return
        # --- USE THE SAME PARSER AS drone_geolocator! ---
        self.srt_data = drone_geolocator.parse_srt_file(srt)
        
                
    def show_frame(self, n):
        if not self.cap: return
        self.cap.set(cv2.CAP_PROP_POS_FRAMES, n-1)
        ret, frame = self.cap.read()
        if not ret: return
        self.current_frame = n
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape; self.frame_width, self.frame_height = w, h
        img = QImage(rgb.data, w, h, ch*w, QImage.Format_RGB888)
        pix = QPixmap.fromImage(img)
    
        painter = QPainter(pix)
        # 1. Draw track markers and trajectories
        if not self.hide_tracks:
            for tr, pts in self.track_points.items():
                show = not self.toggle_only_selected or tr == self.current_track
                for i, (f, (x, y)) in enumerate(pts):
                    if f == n and show:
                        c = QColor(0,255,0) if tr == self.current_track else QColor(255,0,0)
                        painter.setPen(QPen(c,4)); painter.setBrush(c)
                        painter.drawEllipse(QPointF(x,y),12,12)
                    if i > 0 and show:
                        x0, y0 = pts[i-1][1]; painter.setPen(QPen(QColor(255,255,0),4))
                        painter.drawLine(QPointF(x0,y0),QPointF(x,y))
    
        # 2. SRT overlay (only when checkbox is checked)
        if self.overlay_srt_checkbox.isChecked():
            ent = self.srt_data.get(n)
            if ent:
                painter.setPen(QPen(Qt.white))
                painter.setBrush(Qt.NoBrush)
                font = painter.font()
                font.setPointSize(50)
                painter.setFont(font)
                overlay_lines = []
                for k, v in list(ent.items())[:16]:
                    overlay_lines.append(f"{k}: {v}")
                overlay_text = "\n".join(overlay_lines)
                painter.drawText(10, 20, 1000, 1600, Qt.AlignTop | Qt.AlignLeft, overlay_text)

        # 3. Distance measurement overlay (when distance mode is active)
        if self.distance_mode:
            self.draw_distance_overlay(painter)

        painter.end()
    
        self.pixmap_item.setPixmap(pix)
        self.scene.setSceneRect(QRectF(pix.rect()))
        self.slider.blockSignals(True)
        self.slider.setValue(n)
        self.slider.blockSignals(False)
        self.current_frame_display.setText(str(n))
        self.update_marker_summary(redraw_only=True)

    def update_marker_summary(self, redraw_only=False, keep_row=None):
        # keep_row: which marker to keep selected (None=keep previous)
        if keep_row is None:
            sel_marker = self.marker_summary.currentRow()
        else:
            sel_marker = keep_row
        self.marker_summary.blockSignals(True)
        self.marker_summary.clear()
        track = self.current_track
        if not track or track not in self.track_points:
            self.marker_summary.blockSignals(False)
            return
        calf_status = self.track_status.get(track, "without calf")
        points = self.track_points[track]
        for idx, (frame, (x, y)) in enumerate(points):
            mstat = self.marker_statuses.get((track, frame, idx), "underwater")
            txt = f"Frame {frame}, (x={x:.1f}, y={y:.1f}) -- {mstat}"
            self.marker_summary.addItem(txt)
        if sel_marker is not None and 0 <= sel_marker < len(points):
            self.marker_summary.setCurrentRow(sel_marker)
        self.marker_summary.blockSignals(False)

    def marker_selection_changed(self):
        idx = self.marker_summary.currentRow()
        track = self.current_track
        if track and idx >= 0 and track in self.track_points and idx < len(self.track_points[track]):
            frame, (x, y) = self.track_points[track][idx]
            mstat = self.marker_statuses.get((track, frame, idx), "underwater")
            self.status_combo.blockSignals(True)
            self.status_combo.setCurrentText(mstat)
            self.status_combo.blockSignals(False)
            self.show_frame(frame)

    def status_combo_changed(self):
        idx = self.marker_summary.currentRow()
        track = self.current_track
        if track and idx >= 0 and track in self.track_points and idx < len(self.track_points[track]):
            frame, (x, y) = self.track_points[track][idx]
            new_status = self.status_combo.currentText()
            self.marker_statuses[(track, frame, idx)] = new_status
            self.update_marker_summary(keep_row=idx)
          
    def targetEnv_combo_changed(self):
        new_status = self.targetHeighRef.currentText()
        if new_status == 'Marine':
            self.targetEnv = 'm'
        elif new_status == 'Terrestrial':
            self.targetEnv = 't'
            
    # --- DELETE MARKER LOGIC ---
    def delete_selected_marker(self):
        idx = self.marker_summary.currentRow()
        track = self.current_track
        if track and idx >= 0 and track in self.track_points and idx < len(self.track_points[track]):
            frame, (x, y) = self.track_points[track][idx]
            # Remove the marker from track_points
            del self.track_points[track][idx]
            # Rebuild marker statuses for the track, shift indices
            new_marker_statuses = {}
            for i, (f, (px, py)) in enumerate(self.track_points[track]):
                # The marker_statuses key uses the old index, so try to find and remap it
                for j in range(len(self.track_points[track]) + 1):
                    old_key = (track, f, j)
                    if old_key in self.marker_statuses:
                        new_marker_statuses[(track, f, i)] = self.marker_statuses[old_key]
                        break
            # Remove all marker statuses for this track, then update
            for k in list(self.marker_statuses.keys()):
                if k[0] == track:
                    self.marker_statuses.pop(k)
            self.marker_statuses.update(new_marker_statuses)
            # If track is now empty, remove it entirely
            if not self.track_points[track]:
                self.track_points.pop(track)
                self.track_status.pop(track, None)
                self.update_track_list_labels()
                self.current_track = (
                    self.track_list.item(0).text().split(' [', 1)[0]
                    if self.track_list.count() > 0 else ""
                )
            self.update_marker_summary(keep_row=max(0, idx-1))
            self.show_frame(self.current_frame)

    def toggle_play(self):
        if self.timer.isActive():
            self.timer.stop()
            self.play_btn.setText("Play")
        else:
            try:
                step = max(1, int(self.interval_input.text()))
            except:
                step = 1
            self._play_step = step
            self.timer.start(33)
            self.play_btn.setText("Pause")

    def next_frame(self):
        next_idx = min(self.total_frames, self.current_frame + getattr(self, '_play_step', 1))
        self.show_frame(next_idx)

    def slider_changed(self, v): self.show_frame(v)
    
    def handle_overlay_toggle(self):
        self.show_frame(self.current_frame)
    
    
    def goto_frame(self):
        try: self.show_frame(int(self.goto_input.text()))
        except: pass
    def jump_backward(self):
        try: v=int(self.jump_input.text()); self.show_frame(max(1,self.current_frame-v))
        except: pass
    def jump_forward(self):
        try: v=int(self.jump_input.text()); self.show_frame(min(self.total_frames,self.current_frame+v))
        except: pass
    def goto_track_start(self, item):
        track = item.text().split(' [',1)[0]; self.current_track = track
        cat = self.track_status.get(track, self.TRACK_CATEGORIES[0])
        idx = self.TRACK_CATEGORIES.index(cat)
        self.track_category_combo.blockSignals(True)
        self.track_category_combo.setCurrentIndex(idx)
        self.track_category_combo.blockSignals(False)
        self.update_marker_summary()
        
    def remove_selected_track(self):
        it=self.track_list.currentItem()
        if it:
            text = it.text()
            track = text.split(' [', 1)[0]
            self.track_list.takeItem(self.track_list.row(it))
            self.track_points.pop(track,None)
            self.track_status.pop(track,None)
            for k in list(self.marker_statuses.keys()):
                if k[0] == track:
                    self.marker_statuses.pop(k)
            self.current_track = self.track_list.item(0).text().split(' [', 1)[0] if self.track_list.count() > 0 else ""
            self.show_frame(self.current_frame)
            self.update_marker_summary()
            self.update_track_list_labels()         
    def try_parse_float(self,v):
        try:
            # Only parse if it's a pure number (possibly with sign and decimal)
            if isinstance(v, str):
                # Acceptable numeric pattern
                if re.fullmatch(r'[-+]?\d*\.?\d+(e[-+]?\d+)?', v.strip()):
                    return float(v)
            return v
        except Exception:
            return v        
    def latlon_to_xy(self, lat, lon, lat0, lon0):
        """
        Convert lat/lon (deg) to x/y in meters relative to (lat0, lon0)
        Uses simple equirectangular approximation, suitable for small local areas.
        """
        R = 6378137  # earth radius in meters
       
        x = (lon - lon0)*R*np.cos(np.radians((lat+lat0)/2))*(np.pi/180)
        y = (lat - lat0)*R*(np.pi/180)
        
        return x, y

    def convert_geolocation(self):
        # 1. Aggregate track data into a DataFrame
        data = []
        for track, points in self.track_points.items():
            for idx, (frame, (x, y)) in enumerate(points):
                data.append({
                    'track': track,
                    'frame': frame,
                    'x': x,
                    'y': y,
                    'marker_status': self.marker_statuses.get((track, frame, idx), "")
                })
        if not data or not self.srt_data:
            return
    
        tracks_df = pd.DataFrame(data)
    
        # 2. Read camera parameters from UI
        try:
            true_focal_len = float(self.focal_len_input.text())
            pixel_pitch = float(self.pixel_pitch_input.text())
            pp_x = float(self.pp_x_input.text())
            pp_y = float(self.pp_y_input.text())
        except Exception as e:
            print(f"Error reading parameters: {e}")
            return
        
        if self.targetEnv == 'm':
            use_relative_height = False  # target height = 0
        elif self.targetEnv == 't':
            use_relative_height = True  # target height = abs_h - rel_h
            
            
        # 3. Parse SRT metadata to lowercase all keys and ensure correct types for values
        parsed_srt = {}
        for k, meta in self.srt_data.items():
            clean_meta = {}
            for kk, v in meta.items():
                key = kk.lower()
                # If value looks like "95.1 gb_pitch: -41.7 gb_roll: 0.0", split and extract just the number.
                if isinstance(v, str) and re.match(r'^-?\d+(\.\d+)? ', v):
                    v = v.split()[0]
                clean_meta[key] = self.try_parse_float(v)
            parsed_srt[int(k)] = clean_meta
    
        # 4. Call geolocator
        geo_tracks = drone_geolocator.convert_tracks_to_geo(
            tracks_df,
            self.srt_data,  # already correctly parsed
            true_focal_len=true_focal_len,
            pixel_pitch=pixel_pitch,
            img_w=self.vid_width,
            img_h=self.vid_height,
            pp_x=pp_x,
            pp_y=pp_y,
            use_relative_height=use_relative_height
        )
            
        # 5. Plot in the right-bot canvas, in meters
        self.geo_figure.clf()
        ax = self.geo_figure.add_subplot(111)
        # Use the first point as the reference origin for (0,0)
        geo_tracks_valid = geo_tracks.dropna(subset=['latitude', 'longitude'])
        if not geo_tracks_valid.empty:
            first_row = geo_tracks_valid.iloc[0]
            lat0, lon0 = first_row['latitude'], first_row['longitude']
            for name, grp in geo_tracks.groupby('track'):
                xs, ys = [], []
                for idx, row in grp.iterrows():
                    if np.isnan(row['latitude']) or np.isnan(row['longitude']):
                        xs.append(np.nan)
                        ys.append(np.nan)
                    else:
                        x, y = self.latlon_to_xy(row['latitude'], row['longitude'], lat0, lon0)
                        xs.append(x)
                        ys.append(y)
                ax.plot(xs, ys, marker='o', label=name)
            ax.set_title('Tracks (meters from origin)')
            ax.set_xlabel('Easting (m)')
            ax.set_ylabel('Northing (m)')
            ax.legend()
            ax.grid(True)
        else:
            ax.set_title("No valid geolocation data")
        self.geo_canvas.draw()
        self.last_geolocated_tracks = geo_tracks


        
if __name__=="__main__":
    app=QApplication(sys.argv)
    v=VideoViewer()
    v.show()
    sys.exit(app.exec_())
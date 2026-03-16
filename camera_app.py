"""
Quad Hikrobot Camera Capture Application
- Camera 1: MV-CA032-10GM #1 (GigE) - Pass 1 Side View
- Camera 2: MV-CA035 #1       (USB) - Pass 1 Top View
- Camera 3: MV-CA032-10GM #2 (GigE) - Pass 2 Side View
- Camera 4: MV-CA035 #2       (USB) - Pass 2 Top View
Uses Hikrobot MVS SDK (MvCameraControl) for camera control.
"""

import sys
import os
import time
import ctypes
import numpy as np
from datetime import datetime
from threading import Thread, Event

from PySide6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QHBoxLayout, QVBoxLayout,
    QGridLayout, QLabel, QPushButton, QGroupBox, QSplitter, QMessageBox,
    QSizePolicy
)
from PySide6.QtCore import Qt, QTimer, Signal, Slot
from PySide6.QtGui import QImage, QPixmap, QFont

# Hikrobot MVS SDK
from MvCameraControl_class import MvCamera
from CameraParams_header import *
from CameraParams_const import *
from MvErrorDefine_const import *
from PixelType_header import *

# ──────────────────────────────────────────────────────────────────────
#  Output folders
# ──────────────────────────────────────────────────────────────────────
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

PASS1_SIDE = os.path.join(BASE_DIR, "pass one", "side view")
PASS1_TOP  = os.path.join(BASE_DIR, "pass one", "topview")
PASS2_SIDE = os.path.join(BASE_DIR, "pass two", "side view")
PASS2_TOP  = os.path.join(BASE_DIR, "pass two", "topview")

for folder in (PASS1_SIDE, PASS1_TOP, PASS2_SIDE, PASS2_TOP):
    os.makedirs(folder, exist_ok=True)


# ──────────────────────────────────────────────────────────────────────
#  Helper: convert raw buffer to numpy BGR image
# ──────────────────────────────────────────────────────────────────────
def frame_to_numpy(camera, data_buf, frame_info):
    """Convert grabbed frame data to a numpy BGR image using MVS SDK pixel conversion."""
    n_need_size = frame_info.nWidth * frame_info.nHeight * 3
    img_buf = (ctypes.c_ubyte * n_need_size)()

    stConvertParam = MV_CC_PIXEL_CONVERT_PARAM_EX()
    stConvertParam.nWidth = frame_info.nWidth
    stConvertParam.nHeight = frame_info.nHeight
    stConvertParam.pSrcData = data_buf
    stConvertParam.nSrcDataLen = frame_info.nFrameLen
    stConvertParam.enSrcPixelType = frame_info.enPixelType
    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed
    stConvertParam.pDstBuffer = img_buf
    stConvertParam.nDstBufferSize = n_need_size

    ret = camera.MV_CC_ConvertPixelTypeEx(stConvertParam)
    if ret != MV_OK:
        return None

    img_array = np.frombuffer(img_buf, dtype=np.uint8).reshape(
        frame_info.nHeight, frame_info.nWidth, 3
    )
    return img_array


# ──────────────────────────────────────────────────────────────────────
#  Helper: auto-configure GigE camera IP
# ──────────────────────────────────────────────────────────────────────
def auto_configure_gige_ip(device_info, ip_offset=200):
    """
    If the GigE camera is on a different subnet than the network adapter,
    force a compatible IP so the user doesn't need to manually reconfigure.
    ip_offset differentiates multiple cameras on the same adapter.
    """
    gige_info = device_info.SpecialInfo.stGigEInfo
    cam_ip = gige_info.nCurrentIp
    adapter_ip = gige_info.nNetExport
    adapter_mask = gige_info.nCurrentSubNetMask

    if (cam_ip & adapter_mask) == (adapter_ip & adapter_mask):
        return True

    new_ip = (adapter_ip & adapter_mask) | (ip_offset & (~adapter_mask & 0xFFFFFFFF))
    new_mask = adapter_mask
    new_gateway = (adapter_ip & adapter_mask) | (1 & (~adapter_mask & 0xFFFFFFFF))

    cam = MvCamera()
    ret = cam.MV_CC_CreateHandle(device_info)
    if ret != MV_OK:
        return False
    ret = cam.MV_GIGE_ForceIpEx(new_ip, new_mask, new_gateway)
    cam.MV_CC_DestroyHandle()
    return ret == MV_OK


def ip_int_to_str(ip_int):
    return f"{(ip_int >> 24) & 0xFF}.{(ip_int >> 16) & 0xFF}.{(ip_int >> 8) & 0xFF}.{ip_int & 0xFF}"


# ──────────────────────────────────────────────────────────────────────
#  Camera Worker – grabs frames in a background thread
# ──────────────────────────────────────────────────────────────────────
class CameraWorker(Thread):
    def __init__(self, camera, buf_size):
        super().__init__(daemon=True)
        self.camera = camera
        self.buf_size = buf_size
        self.stop_event = Event()
        self.latest_frame = None

    def run(self):
        data_buf = (ctypes.c_ubyte * self.buf_size)()
        frame_info = MV_FRAME_OUT_INFO_EX()
        while not self.stop_event.is_set():
            ret = self.camera.MV_CC_GetOneFrameTimeout(
                data_buf, self.buf_size, frame_info, 1000
            )
            if ret == MV_OK:
                img = frame_to_numpy(self.camera, data_buf, frame_info)
                if img is not None:
                    self.latest_frame = img.copy()

    def stop(self):
        self.stop_event.set()


# ──────────────────────────────────────────────────────────────────────
#  Main Window
# ──────────────────────────────────────────────────────────────────────
class MainWindow(QMainWindow):
    sig_p1_side = Signal(np.ndarray)
    sig_p1_top  = Signal(np.ndarray)
    sig_p2_side = Signal(np.ndarray)
    sig_p2_top  = Signal(np.ndarray)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Bushing Washer – Quad Camera Inspection")
        self.setMinimumSize(1280, 720)
        self.showMaximized()

        self.cams = {
            "p1_gige": None, "p1_usb": None,
            "p2_gige": None, "p2_usb": None,
        }
        self.workers = {
            "p1_gige": None, "p1_usb": None,
            "p2_gige": None, "p2_usb": None,
        }
        self.streaming = False
        self.capture_counter = 0

        self._build_ui()
        self._connect_signals()

        self.frame_timer = QTimer(self)
        self.frame_timer.timeout.connect(self._refresh_frames)

    # ── UI ───────────────────────────────────────────────────────────
    def _build_ui(self):
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QHBoxLayout(central)
        main_layout.setContentsMargins(6, 6, 6, 6)

        # ── Left: 2x2 camera grid (70%) ─────────────────────────────
        feeds_widget = QWidget()
        grid = QGridLayout(feeds_widget)
        grid.setContentsMargins(0, 0, 0, 0)
        grid.setSpacing(6)

        feed_style = "background-color: #1e1e1e; color: #888;"

        # Row 0: Pass 1  (Top View left, Side View right)
        grp = QGroupBox("Pass 1 – Top View (USB #1)")
        lay = QVBoxLayout(grp)
        self.lbl_p1_top = QLabel("No Stream")
        self.lbl_p1_top.setAlignment(Qt.AlignCenter)
        self.lbl_p1_top.setStyleSheet(feed_style)
        self.lbl_p1_top.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self.lbl_p1_top)
        grid.addWidget(grp, 0, 0)

        grp = QGroupBox("Pass 1 – Side View (GigE #1)")
        lay = QVBoxLayout(grp)
        self.lbl_p1_side = QLabel("No Stream")
        self.lbl_p1_side.setAlignment(Qt.AlignCenter)
        self.lbl_p1_side.setStyleSheet(feed_style)
        self.lbl_p1_side.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self.lbl_p1_side)
        grid.addWidget(grp, 0, 1)

        # Row 1: Pass 2  (Top View left, Side View right)
        grp = QGroupBox("Pass 2 – Top View (USB #2)")
        lay = QVBoxLayout(grp)
        self.lbl_p2_top = QLabel("No Stream")
        self.lbl_p2_top.setAlignment(Qt.AlignCenter)
        self.lbl_p2_top.setStyleSheet(feed_style)
        self.lbl_p2_top.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self.lbl_p2_top)
        grid.addWidget(grp, 1, 0)

        grp = QGroupBox("Pass 2 – Side View (GigE #2)")
        lay = QVBoxLayout(grp)
        self.lbl_p2_side = QLabel("No Stream")
        self.lbl_p2_side.setAlignment(Qt.AlignCenter)
        self.lbl_p2_side.setStyleSheet(feed_style)
        self.lbl_p2_side.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        lay.addWidget(self.lbl_p2_side)
        grid.addWidget(grp, 1, 1)

        # ── Right: control panel (30%) ───────────────────────────────
        control_widget = QWidget()
        ctrl = QVBoxLayout(control_widget)
        ctrl.setContentsMargins(10, 10, 10, 10)
        ctrl.setSpacing(14)

        title = QLabel("Controls")
        title.setFont(QFont("Segoe UI", 16, QFont.Bold))
        title.setAlignment(Qt.AlignCenter)
        ctrl.addWidget(title)

        # Camera status
        info_group = QGroupBox("Camera Status")
        info_lay = QVBoxLayout(info_group)
        self.lbl_status = {}
        for key, label_text in [
            ("p1_gige", "Pass 1 GigE (Side): not connected"),
            ("p1_usb",  "Pass 1 USB  (Top) : not connected"),
            ("p2_gige", "Pass 2 GigE (Side): not connected"),
            ("p2_usb",  "Pass 2 USB  (Top) : not connected"),
        ]:
            lbl = QLabel(label_text)
            lbl.setWordWrap(True)
            info_lay.addWidget(lbl)
            self.lbl_status[key] = lbl
        ctrl.addWidget(info_group)

        # Buttons
        btn_style = """
            QPushButton {
                padding: 14px; font-size: 15px; font-weight: bold; border-radius: 6px;
            }
            QPushButton:disabled { background-color: #555; color: #999; }
        """
        self.btn_start = QPushButton("▶  Start Stream")
        self.btn_start.setStyleSheet(btn_style + """
            QPushButton:enabled { background-color: #2e7d32; color: white; }
            QPushButton:enabled:hover { background-color: #388e3c; }
        """)
        ctrl.addWidget(self.btn_start)

        self.btn_capture = QPushButton("📷  Capture")
        self.btn_capture.setEnabled(False)
        self.btn_capture.setStyleSheet(btn_style + """
            QPushButton:enabled { background-color: #1565c0; color: white; }
            QPushButton:enabled:hover { background-color: #1976d2; }
        """)
        ctrl.addWidget(self.btn_capture)

        self.btn_stop = QPushButton("⏹  Stop Stream")
        self.btn_stop.setEnabled(False)
        self.btn_stop.setStyleSheet(btn_style + """
            QPushButton:enabled { background-color: #c62828; color: white; }
            QPushButton:enabled:hover { background-color: #d32f2f; }
        """)
        ctrl.addWidget(self.btn_stop)

        self.lbl_count = QLabel("Captures: 0")
        self.lbl_count.setFont(QFont("Segoe UI", 13))
        self.lbl_count.setAlignment(Qt.AlignCenter)
        ctrl.addWidget(self.lbl_count)

        # Save paths
        paths_group = QGroupBox("Save Locations")
        paths_lay = QVBoxLayout(paths_group)
        paths_lay.addWidget(QLabel(f"Pass 1 Side → {PASS1_SIDE}"))
        paths_lay.addWidget(QLabel(f"Pass 1 Top  → {PASS1_TOP}"))
        paths_lay.addWidget(QLabel(f"Pass 2 Side → {PASS2_SIDE}"))
        paths_lay.addWidget(QLabel(f"Pass 2 Top  → {PASS2_TOP}"))
        ctrl.addWidget(paths_group)

        ctrl.addStretch()

        # ── Splitter 70/30 ───────────────────────────────────────────
        splitter = QSplitter(Qt.Horizontal)
        splitter.addWidget(feeds_widget)
        splitter.addWidget(control_widget)
        splitter.setStretchFactor(0, 7)
        splitter.setStretchFactor(1, 3)
        splitter.setHandleWidth(4)
        main_layout.addWidget(splitter)

        self.statusBar().showMessage("Ready – click Start Stream to begin.")

    # ── Signals ──────────────────────────────────────────────────────
    def _connect_signals(self):
        self.btn_start.clicked.connect(self._on_start)
        self.btn_capture.clicked.connect(self._on_capture)
        self.btn_stop.clicked.connect(self._on_stop)
        self.sig_p1_side.connect(lambda img: self._set_pixmap(self.lbl_p1_side, img))
        self.sig_p1_top.connect(lambda img: self._set_pixmap(self.lbl_p1_top, img))
        self.sig_p2_side.connect(lambda img: self._set_pixmap(self.lbl_p2_side, img))
        self.sig_p2_top.connect(lambda img: self._set_pixmap(self.lbl_p2_top, img))

    def _set_pixmap(self, label, img):
        h, w, ch = img.shape
        rgb = img[:, :, ::-1].copy()
        qimg = QImage(rgb.data, w, h, w * ch, QImage.Format_RGB888)
        pix = QPixmap.fromImage(qimg)
        label.setPixmap(pix.scaled(label.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

    # ── Enumerate & open cameras ─────────────────────────────────────
    def _find_and_open_cameras(self):
        print("[INFO] Enumerating cameras...")

        # Enumerate GigE
        gige_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, gige_list)
        print(f"[INFO] GigE enum ret=0x{ret:08X}, found={gige_list.nDeviceNum}")
        if ret != MV_OK:
            gige_list.nDeviceNum = 0

        # Enumerate USB
        usb_list = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, usb_list)
        print(f"[INFO] USB enum ret=0x{ret:08X}, found={usb_list.nDeviceNum}")
        if ret != MV_OK:
            usb_list.nDeviceNum = 0

        # Collect all GigE device infos
        gige_devices = []
        for i in range(gige_list.nDeviceNum):
            dev = ctypes.cast(
                gige_list.pDeviceInfo[i], ctypes.POINTER(MV_CC_DEVICE_INFO)
            ).contents
            if dev.nTLayerType == MV_GIGE_DEVICE:
                gige_devices.append(dev)

        # Collect all USB device infos
        usb_devices = []
        for i in range(usb_list.nDeviceNum):
            dev = ctypes.cast(
                usb_list.pDeviceInfo[i], ctypes.POINTER(MV_CC_DEVICE_INFO)
            ).contents
            if dev.nTLayerType == MV_USB_DEVICE:
                usb_devices.append(dev)

        print(f"[INFO] GigE devices: {len(gige_devices)}, USB devices: {len(usb_devices)}")

        # Auto-configure GigE IPs (different offsets to avoid conflicts)
        for idx, dev in enumerate(gige_devices):
            auto_configure_gige_ip(dev, ip_offset=200 + idx)

        if gige_devices:
            time.sleep(0.5)
            # Re-enumerate after ForceIP
            gige_list2 = MV_CC_DEVICE_INFO_LIST()
            MvCamera.MV_CC_EnumDevices(MV_GIGE_DEVICE, gige_list2)
            gige_devices = []
            for i in range(gige_list2.nDeviceNum):
                dev = ctypes.cast(
                    gige_list2.pDeviceInfo[i], ctypes.POINTER(MV_CC_DEVICE_INFO)
                ).contents
                if dev.nTLayerType == MV_GIGE_DEVICE:
                    gige_devices.append(dev)

        # Open cameras: Pass 1 = index 0, Pass 2 = index 1
        cam_map = [
            ("p1_gige", gige_devices, 0, "GigE"),
            ("p1_usb",  usb_devices,  0, "USB"),
            ("p2_gige", gige_devices, 1, "GigE"),
            ("p2_usb",  usb_devices,  1, "USB"),
        ]

        for key, dev_list, idx, transport in cam_map:
            if idx >= len(dev_list):
                self.lbl_status[key].setText(f"{key}: no camera (need {transport} #{idx+1})")
                print(f"[WARN] {key}: {transport} camera #{idx+1} not found")
                continue

            dev = dev_list[idx]
            cam = MvCamera()
            ret = cam.MV_CC_CreateHandle(dev)
            if ret != MV_OK:
                self.lbl_status[key].setText(f"{key}: CreateHandle failed 0x{ret:08X}")
                print(f"[ERROR] {key} CreateHandle: 0x{ret:08X}")
                continue

            if transport == "GigE":
                ret = cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive)
            else:
                ret = cam.MV_CC_OpenDevice()

            if ret != MV_OK:
                cam.MV_CC_DestroyHandle()
                self.lbl_status[key].setText(f"{key}: OpenDevice failed 0x{ret:08X}")
                print(f"[ERROR] {key} OpenDevice: 0x{ret:08X}")
                continue

            # Camera uses whatever settings are currently active.
            # To persist settings across power cycles, save them in MVS Studio:
            #   UserSet Control → UserSetSelector = Default → UserSetSave

            if transport == "GigE":
                pkt = cam.MV_CC_GetOptimalPacketSize()
                if pkt > 0:
                    cam.MV_CC_SetIntValueEx("GevSCPSPacketSize", pkt)

            self.cams[key] = cam

            if transport == "GigE":
                gi = dev.SpecialInfo.stGigEInfo
                model = "".join(chr(c) for c in gi.chModelName if c != 0)
                ip = ip_int_to_str(gi.nCurrentIp)
                self.lbl_status[key].setText(f"{key}: {model}  IP:{ip} ✓")
            else:
                ui = dev.SpecialInfo.stUsb3VInfo
                model = "".join(chr(c) for c in ui.chModelName if c != 0)
                self.lbl_status[key].setText(f"{key}: {model} ✓")
            print(f"[INFO] {key} opened successfully")

    # ── Start streaming ──────────────────────────────────────────────
    def _on_start(self):
        self.statusBar().showMessage("Enumerating cameras...")
        QApplication.processEvents()

        try:
            self._find_and_open_cameras()
        except Exception as e:
            print(f"[ERROR] _find_and_open_cameras exception: {e}")
            import traceback; traceback.print_exc()
            QMessageBox.critical(self, "Error", f"Camera init failed:\n{e}")
            return

        any_opened = any(c is not None for c in self.cams.values())
        if not any_opened:
            QMessageBox.warning(
                self, "No Cameras",
                "No Hikrobot cameras were detected.\n\n"
                "• Check cables and power.\n"
                "• Make sure MVS runtime / SDK is installed.\n"
                "• Close MVS Studio if it's holding the cameras.",
            )
            return

        for key, cam in self.cams.items():
            if cam is None:
                continue
            ret = cam.MV_CC_StartGrabbing()
            print(f"[INFO] {key} StartGrabbing ret=0x{ret:08X}")
            if ret != MV_OK:
                self.statusBar().showMessage(f"{key} StartGrabbing failed: 0x{ret:08X}")
                continue
            stParam = MVCC_INTVALUE_EX()
            cam.MV_CC_GetIntValueEx("PayloadSize", stParam)
            buf_size = int(stParam.nCurValue) + 2048
            print(f"[INFO] {key} payload buf_size={buf_size}")
            worker = CameraWorker(cam, buf_size)
            worker.start()
            self.workers[key] = worker

        self.streaming = True
        self.frame_timer.start(33)
        self.btn_start.setEnabled(False)
        self.btn_capture.setEnabled(True)
        self.btn_stop.setEnabled(True)
        self.statusBar().showMessage("Streaming...")

    # ── Refresh frames ───────────────────────────────────────────────
    def _refresh_frames(self):
        sig_map = {
            "p1_gige": self.sig_p1_side,
            "p1_usb":  self.sig_p1_top,
            "p2_gige": self.sig_p2_side,
            "p2_usb":  self.sig_p2_top,
        }
        for key, signal in sig_map.items():
            w = self.workers[key]
            if w and w.latest_frame is not None:
                signal.emit(w.latest_frame)

    # ── Capture ──────────────────────────────────────────────────────
    def _on_capture(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        self.capture_counter += 1
        saved = []

        save_map = {
            "p1_gige": (PASS1_SIDE, f"p1_side_{timestamp}.bmp"),
            "p1_usb":  (PASS1_TOP,  f"p1_top_{timestamp}.bmp"),
            "p2_gige": (PASS2_SIDE, f"p2_side_{timestamp}.bmp"),
            "p2_usb":  (PASS2_TOP,  f"p2_top_{timestamp}.bmp"),
        }

        for key, (folder, filename) in save_map.items():
            w = self.workers[key]
            if w and w.latest_frame is not None:
                frame = w.latest_frame.copy()
                path = os.path.join(folder, filename)
                self._save_bgr_bmp(frame, path)
                saved.append(key)

        self.lbl_count.setText(f"Captures: {self.capture_counter}")
        if saved:
            self.statusBar().showMessage(
                f"Captured #{self.capture_counter}: {', '.join(saved)}  ({timestamp})"
            )
        else:
            self.statusBar().showMessage("Capture failed – no frames available.")

    @staticmethod
    def _save_bgr_bmp(bgr_img, filepath):
        """Save a BGR numpy image as BMP without needing OpenCV."""
        h, w, ch = bgr_img.shape
        row_size = (w * 3 + 3) & ~3
        pixel_data_size = row_size * h
        file_size = 54 + pixel_data_size

        padded = np.zeros((h, row_size), dtype=np.uint8)
        for y in range(h):
            row_bytes = bgr_img[h - 1 - y].tobytes()
            padded[y, :w * 3] = np.frombuffer(row_bytes, dtype=np.uint8)

        header = bytearray(54)
        header[0:2] = b'BM'
        header[2:6] = file_size.to_bytes(4, 'little')
        header[10:14] = (54).to_bytes(4, 'little')
        header[14:18] = (40).to_bytes(4, 'little')
        header[18:22] = w.to_bytes(4, 'little')
        header[22:26] = h.to_bytes(4, 'little')
        header[26:28] = (1).to_bytes(2, 'little')
        header[28:30] = (24).to_bytes(2, 'little')
        header[34:38] = pixel_data_size.to_bytes(4, 'little')

        with open(filepath, 'wb') as f:
            f.write(header)
            f.write(padded.tobytes())

    # ── Stop streaming ───────────────────────────────────────────────
    def _on_stop(self):
        self.frame_timer.stop()

        for key in self.workers:
            w = self.workers[key]
            if w:
                w.stop()
                w.join(timeout=3)
                self.workers[key] = None

        for key in self.cams:
            cam = self.cams[key]
            if cam:
                cam.MV_CC_StopGrabbing()
                cam.MV_CC_CloseDevice()
                cam.MV_CC_DestroyHandle()
                self.cams[key] = None

        self.streaming = False
        self.btn_start.setEnabled(True)
        self.btn_capture.setEnabled(False)
        self.btn_stop.setEnabled(False)

        for lbl in (self.lbl_p1_side, self.lbl_p1_top, self.lbl_p2_side, self.lbl_p2_top):
            lbl.setPixmap(QPixmap())
            lbl.setText("No Stream")

        for key in self.lbl_status:
            self.lbl_status[key].setText(f"{key}: not connected")

        self.statusBar().showMessage("Stopped.")

    def closeEvent(self, event):
        if self.streaming:
            self._on_stop()
        event.accept()


# ──────────────────────────────────────────────────────────────────────
def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    from PySide6.QtGui import QPalette, QColor
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(45, 45, 45))
    palette.setColor(QPalette.WindowText, QColor(220, 220, 220))
    palette.setColor(QPalette.Base, QColor(30, 30, 30))
    palette.setColor(QPalette.AlternateBase, QColor(45, 45, 45))
    palette.setColor(QPalette.ToolTipBase, QColor(220, 220, 220))
    palette.setColor(QPalette.ToolTipText, QColor(220, 220, 220))
    palette.setColor(QPalette.Text, QColor(220, 220, 220))
    palette.setColor(QPalette.Button, QColor(55, 55, 55))
    palette.setColor(QPalette.ButtonText, QColor(220, 220, 220))
    palette.setColor(QPalette.BrightText, QColor(255, 50, 50))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, QColor(0, 0, 0))
    app.setPalette(palette)

    win = MainWindow()
    win.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()

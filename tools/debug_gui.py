"""
STM32H743 + OV7670 serial debug GUI.

Features:
  - Connect to ST-Link VCP / serial port
  - Send single-key field-debug commands to firmware
  - Live UART log view
  - Receive raw FRAME dumps and preview 4 decode variants

Requirements:
    pip install pyserial pillow
"""
from __future__ import annotations

import queue
import struct
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import ttk

import re

import serial
import serial.tools.list_ports
from PIL import Image, ImageDraw, ImageTk

BAUD = 921600
FRAME_MARKER = b"FRAME"
FRAME_TAIL = b"END\n"
MAX_FRAME_BYTES = 1_000_000
MAIN_PREVIEW_MAX = (760, 570)
THUMB_PREVIEW_MAX = (320, 240)
AUTO_SETTLE_MS = 900
AUTO_FRAME_TIMEOUT_MS = 6000
LIVE_DEFAULT_INTERVAL_MS = 1800

try:
    RESAMPLING = Image.Resampling
except AttributeError:  # Pillow < 9.1
    RESAMPLING = Image


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


def default_port():
    ports = list(serial.tools.list_ports.comports())
    for p in ports:
        desc = (p.description or "").lower()
        if "stlink" in desc or "st-link" in desc or "virtual com" in desc:
            return p.device
    return ports[0].device if ports else ""


def rgb565_to_image(data: bytes, width: int, height: int) -> Image.Image:
    """Decode RGB565 big-endian (OV7670 DCMI native byte order: MSB first)."""
    rgb = bytearray(width * height * 3)
    for i in range(width * height):
        px = (data[i * 2] << 8) | data[i * 2 + 1]
        rgb[i * 3 + 0] = ((px >> 11) & 0x1F) * 255 // 31
        rgb[i * 3 + 1] = ((px >> 5) & 0x3F) * 255 // 63
        rgb[i * 3 + 2] = (px & 0x1F) * 255 // 31
    return Image.frombytes("RGB", (width, height), bytes(rgb))


def byteswap_16(data: bytes) -> bytes:
    swapped = bytearray(len(data))
    swapped[0::2] = data[1::2]
    swapped[1::2] = data[0::2]
    return bytes(swapped)


def wordswap_32(data: bytes) -> bytes:
    swapped = bytearray(data)
    limit = len(data) - (len(data) % 4)
    for i in range(0, limit, 4):
        swapped[i:i + 4] = data[i + 2:i + 4] + data[i:i + 2]
    return bytes(swapped)


def frame_variants(pixel_data: bytes) -> dict[str, bytes]:
    return {
        "Normal (BE)": pixel_data,
        "Byte Swapped (LE)": byteswap_16(pixel_data),
        "Word Swapped": wordswap_32(pixel_data),
        "Word+Byte Swapped": byteswap_16(wordswap_32(pixel_data)),
    }


def fit_image(image: Image.Image, max_size: tuple[int, int], resample) -> Image.Image:
    max_w, max_h = max_size
    scale = min(max_w / image.width, max_h / image.height)
    scale = max(scale, 1e-3)
    target = (max(1, int(image.width * scale)), max(1, int(image.height * scale)))
    return image.resize(target, resample)


FILE_MAP = {
    "Normal (BE)": "frame_normal.png",
    "Byte Swapped (LE)": "frame_swapped.png",
    "Word Swapped": "frame_wordswapped.png",
    "Word+Byte Swapped": "frame_word_byteswapped.png",
}


class SerialWorker(threading.Thread):
    def __init__(self, port: str, baud: int, event_queue: queue.Queue):
        super().__init__(daemon=True)
        self.port = port
        self.baud = baud
        self.event_queue = event_queue
        self.stop_event = threading.Event()
        self.serial = None
        self.rx_buf = bytearray()
        self.tx_lock = threading.Lock()

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baud, timeout=0.05)
            self.serial.reset_input_buffer()
            self.event_queue.put(("status", f"Connected: {self.port} @ {self.baud}"))
        except Exception as exc:
            self.event_queue.put(("error", f"Open failed: {exc}"))
            return

        try:
            while not self.stop_event.is_set():
                chunk = self.serial.read(512)
                if chunk:
                    self.rx_buf.extend(chunk)
                    self._process_rx()
                else:
                    time.sleep(0.01)
        finally:
            try:
                if self.serial is not None:
                    self.serial.close()
            finally:
                self.event_queue.put(("status", "Disconnected"))

    def stop(self):
        self.stop_event.set()

    def send_command(self, cmd: str):
        if not self.serial or not self.serial.is_open:
            raise RuntimeError("Serial port is not connected.")
        with self.tx_lock:
            self.serial.write(cmd.encode("ascii"))

    def _emit_log(self, data: bytes):
        if not data:
            return
        text = data.decode("utf-8", errors="replace")
        if text:
            self.event_queue.put(("log", text))

    def _pending_frame_prefix_len(self) -> int:
        max_keep = min(len(self.rx_buf), len(FRAME_MARKER) - 1)
        for keep in range(max_keep, 0, -1):
            if self.rx_buf[-keep:] == FRAME_MARKER[:keep]:
                return keep
        return 0

    def _process_rx(self):
        while True:
            idx = self.rx_buf.find(FRAME_MARKER)
            if idx < 0:
                keep = self._pending_frame_prefix_len()
                if len(self.rx_buf) > keep:
                    if keep > 0:
                        flush = bytes(self.rx_buf[:-keep])
                        del self.rx_buf[:-keep]
                    else:
                        flush = bytes(self.rx_buf)
                        self.rx_buf.clear()
                    self._emit_log(flush)
                return

            if idx > 0:
                prefix = bytes(self.rx_buf[:idx])
                del self.rx_buf[:idx]
                self._emit_log(prefix)
                continue

            if len(self.rx_buf) < 10:
                return

            width = struct.unpack(">H", self.rx_buf[5:7])[0]
            height = struct.unpack(">H", self.rx_buf[7:9])[0]
            bpp = self.rx_buf[9]
            total = width * height * bpp

            if bpp not in (1, 2, 3, 4) or total <= 0 or total > MAX_FRAME_BYTES:
                self._emit_log(bytes(self.rx_buf[:5]))
                del self.rx_buf[:5]
                continue

            frame_packet_size = 10 + total
            if len(self.rx_buf) < frame_packet_size:
                return

            pixel_data = bytes(self.rx_buf[10:frame_packet_size])
            del self.rx_buf[:frame_packet_size]

            if self.rx_buf.startswith(FRAME_TAIL):
                del self.rx_buf[: len(FRAME_TAIL)]

            self.event_queue.put(("frame", (width, height, bpp, pixel_data)))


class DebugGui:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("OV7670 Debug Tool")
        self.root.geometry("1280x1050")
        self.root.minsize(900, 700)

        self.events = queue.Queue()
        self.worker: SerialWorker | None = None
        self.image_refs = {}
        self.latest_variant_images: dict[str, Image.Image] = {}
        self.latest_window_lines: list[str] = []
        self.auto_active = False
        self.auto_pending_shift: int | None = None
        self.auto_max_shift = 0
        self.auto_timeout_token = 0
        self.auto_run_dir: Path | None = None
        self.live_active = False
        self.live_waiting_frame = False
        self.live_after_id: str | None = None

        self.port_var = tk.StringVar(value=default_port())
        self.status_var = tk.StringVar(value="Idle")
        self.frame_var = tk.StringVar(value="No frame yet")
        self.auto_steps_var = tk.IntVar(value=4)
        self.live_interval_var = tk.IntVar(value=LIVE_DEFAULT_INTERVAL_MS)
        self.main_variant_var = tk.StringVar(value="Normal (BE)")
        self.track_var = tk.StringVar(value="Tracking: --")
        self.pixel_var = tk.StringVar(value="Click image to inspect pixel")
        self.track_info: dict | None = None  # latest parsed [TRACK] data
        self.track_local = tk.BooleanVar(value=False)  # GUI-side detection toggle
        self.show_mask = tk.BooleanVar(value=False)     # red mask overlay
        # HSV thresholds: red hue is near 0/360, so we use two ranges
        self.tr_h_lo = tk.IntVar(value=340)    # hue lower bound (red wraps around 0)
        self.tr_h_hi = tk.IntVar(value=30)     # hue upper bound
        self.tr_s_min = tk.IntVar(value=30)    # min saturation [0-255]
        self.tr_v_min = tk.IntVar(value=50)    # min value [0-255]
        self.tr_min_px = tk.IntVar(value=60)
        self.latest_raw_frame: tuple[int, int, bytes] | None = None  # (w, h, pixel_data)

        # OV7670 camera tuning sliders (register, default)
        self.cam_r_gain = tk.IntVar(value=0x40)      # RED  (reg 0x02)
        self.cam_b_gain = tk.IntVar(value=0x60)      # BLUE (reg 0x01)
        self.cam_g_gain = tk.IntVar(value=0x40)      # GREEN (reg 0x6A)
        self.cam_saturation = tk.IntVar(value=0x80)   # Sat  (reg 0xC9)
        self.cam_brightness = tk.IntVar(value=0x00)   # Bright (reg 0x55)
        self.cam_contrast = tk.IntVar(value=0x40)     # Contrast (reg 0x56)
        self.cam_mtx1 = tk.IntVar(value=0xC0)         # MTX1 R gain (reg 0x4F)
        self.cam_mtx2 = tk.IntVar(value=0x90)         # MTX2 G-to-R (reg 0x50)
        self.cam_mtx5 = tk.IntVar(value=0xB0)         # MTX5 G gain (reg 0x53)
        self.cam_mtx6 = tk.IntVar(value=0xE3)         # MTX6 B gain (reg 0x54)
        self.win_lines: list[str] = []

        self._build_ui()
        self.refresh_ports()
        self.root.after(50, self.poll_events)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def _build_ui(self):
        root = self.root

        # --- Scrollable container ---
        self._outer = ttk.Frame(root)
        self._outer.pack(fill="both", expand=True)

        self._canvas = tk.Canvas(self._outer, highlightthickness=0)
        self._v_scroll = ttk.Scrollbar(self._outer, orient="vertical", command=self._canvas.yview)
        self._h_scroll = ttk.Scrollbar(self._outer, orient="horizontal", command=self._canvas.xview)
        self._canvas.configure(yscrollcommand=self._v_scroll.set, xscrollcommand=self._h_scroll.set)

        self._v_scroll.pack(side="right", fill="y")
        self._h_scroll.pack(side="bottom", fill="x")
        self._canvas.pack(side="left", fill="both", expand=True)

        self._inner = ttk.Frame(self._canvas)
        self._canvas_win = self._canvas.create_window((0, 0), window=self._inner, anchor="nw")

        self._inner.bind("<Configure>", self._on_inner_configure)
        self._canvas.bind("<Configure>", self._on_canvas_configure)
        # Mouse wheel scrolling
        self._canvas.bind_all("<MouseWheel>", self._on_mousewheel)

        container = self._inner
        container.columnconfigure(0, weight=1)
        container.rowconfigure(2, weight=3)
        container.rowconfigure(5, weight=2)

        # --- Top bar: COM port controls ---
        top = ttk.Frame(container, padding=8)
        top.grid(row=0, column=0, sticky="ew")
        top.columnconfigure(7, weight=1)

        ttk.Label(top, text="COM").grid(row=0, column=0, sticky="w")
        self.port_combo = ttk.Combobox(top, textvariable=self.port_var, width=14, state="readonly")
        self.port_combo.grid(row=0, column=1, padx=(4, 8))

        ttk.Button(top, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=4)
        ttk.Button(top, text="Connect", command=self.connect).grid(row=0, column=3, padx=4)
        ttk.Button(top, text="Disconnect", command=self.disconnect).grid(row=0, column=4, padx=4)

        ttk.Label(top, text=f"Baud {BAUD}").grid(row=0, column=5, padx=(12, 4))
        ttk.Label(top, textvariable=self.status_var).grid(row=0, column=7, sticky="e")

        # --- Firmware Control: use flow-wrap for buttons ---
        controls = ttk.LabelFrame(container, text="Firmware Control", padding=8)
        controls.grid(row=1, column=0, sticky="ew", padx=8, pady=(0, 8))
        controls.columnconfigure(0, weight=1)

        # Row 0: command buttons in a flow-wrap frame
        btn_flow = ttk.Frame(controls)
        btn_flow.grid(row=0, column=0, sticky="ew")
        btn_flow.columnconfigure(0, weight=1)

        self._btn_flow_inner = ttk.Frame(btn_flow)
        self._btn_flow_inner.pack(anchor="w")

        buttons = [
            ("Preview (p)", "p"),
            ("Color Bar (c)", "c"),
            ("Tracking (t)", "t"),
            ("Diag (d)", "d"),
            ("Window/Freeze (w)", "w"),
            ("Reset Baseline (x)", "x"),
            ("PCLK Rise (r)", "r"),
            ("PCLK Fall (f)", "f"),
            ("LCD SPI /8", "8"),
            ("LCD SPI /4", "4"),
            ("Byte Swap (b)", "b"),
            ("H- (j)", "j"),
            ("H+ (l)", "l"),
            ("Sharp- (q)", "q"),
            ("Sharp+ (e)", "e"),
            ("Noise- (a)", "a"),
            ("Noise+ (s)", "s"),
            ("Sensor Swap (m)", "m"),
            ("Shift - ([)", "["),
            ("Shift + (])", "]"),
            ("Dump Frame (u)", "u"),
            ("Help (h)", "h"),
        ]
        self._cmd_buttons = []
        for label, cmd in buttons:
            btn = ttk.Button(self._btn_flow_inner, text=label,
                             command=lambda c=cmd: self.send_command(c))
            self._cmd_buttons.append(btn)

        # Row 1: auto sweep + live grab controls
        row1 = ttk.Frame(controls)
        row1.grid(row=1, column=0, sticky="ew", pady=(6, 0))

        ttk.Label(row1, text="Auto max shift").pack(side="left", padx=(0, 2))
        ttk.Spinbox(
            row1, from_=0, to=16, width=5,
            textvariable=self.auto_steps_var,
        ).pack(side="left", padx=(0, 4))
        ttk.Button(row1, text="Auto Sweep", command=self.start_auto_sweep).pack(
            side="left", padx=4)
        ttk.Label(row1, text="Runs: c -> z -> dump shift0..N").pack(
            side="left", padx=(4, 16))

        ttk.Separator(row1, orient="vertical").pack(side="left", fill="y", padx=8)

        self.live_button = ttk.Button(row1, text="Start Live Grab",
                                      command=self.toggle_live_capture)
        self.live_button.pack(side="left", padx=4)
        ttk.Label(row1, text="Interval ms").pack(side="left", padx=(8, 2))
        ttk.Spinbox(
            row1, from_=400, to=10000, increment=100, width=6,
            textvariable=self.live_interval_var,
        ).pack(side="left", padx=(0, 4))

        # Bind resize to reflow buttons
        btn_flow.bind("<Configure>", self._reflow_buttons)
        self._btn_flow_container = btn_flow

        # --- Image preview area ---
        images = ttk.Frame(container, padding=8)
        images.grid(row=2, column=0, sticky="nsew")
        images.columnconfigure(0, weight=3)
        images.columnconfigure(1, weight=2)
        images.rowconfigure(0, weight=1)

        main_frame = ttk.LabelFrame(images, text="Camera View", padding=8)
        main_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 8))
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)

        decode_bar = ttk.Frame(main_frame)
        decode_bar.grid(row=0, column=0, sticky="w", pady=(0, 6))
        ttk.Label(decode_bar, text="Main decode").grid(row=0, column=0, padx=(0, 8))
        for i, title in enumerate(FILE_MAP.keys(), start=1):
            ttk.Radiobutton(
                decode_bar,
                text=title,
                value=title,
                variable=self.main_variant_var,
                command=self.refresh_preview_images,
            ).grid(row=0, column=i, padx=4)

        self.main_preview_label = ttk.Label(main_frame, anchor="center")
        self.main_preview_label.grid(row=1, column=0, sticky="nsew")
        self.main_preview_label.bind("<Button-1>", self._on_preview_click)

        thumb_frame = ttk.LabelFrame(images, text="Decode Variants", padding=8)
        thumb_frame.grid(row=0, column=1, sticky="nsew")
        thumb_frame.columnconfigure(0, weight=1)
        thumb_frame.columnconfigure(1, weight=1)
        thumb_frame.rowconfigure(1, weight=1)
        thumb_frame.rowconfigure(3, weight=1)

        self.preview_labels = {}
        preview_specs = [
            ("Normal (BE)", 0, 0),
            ("Byte Swapped (LE)", 0, 1),
            ("Word Swapped", 2, 0),
            ("Word+Byte Swapped", 2, 1),
        ]
        for title, row, col in preview_specs:
            ttk.Label(thumb_frame, text=title).grid(row=row, column=col)
            label = ttk.Label(thumb_frame, anchor="center")
            label.grid(row=row + 1, column=col, sticky="nsew", padx=6, pady=(0, 8))
            self.preview_labels[title] = label

        # --- Tracking threshold controls ---
        track_frame = ttk.LabelFrame(container, text="Red Tracking Threshold", padding=6)
        track_frame.grid(row=3, column=0, sticky="ew", padx=8, pady=(0, 4))

        # Row 0: controls
        # Row 0: checkboxes + buttons + status
        tr_row0 = ttk.Frame(track_frame)
        tr_row0.pack(fill="x")

        ttk.Checkbutton(
            tr_row0, text="GUI detect",
            variable=self.track_local,
            command=self._on_track_param_changed,
        ).pack(side="left", padx=(0, 6))

        ttk.Checkbutton(
            tr_row0, text="Show Mask",
            variable=self.show_mask,
            command=self._on_track_param_changed,
        ).pack(side="left", padx=(0, 12))

        ttk.Button(
            tr_row0, text="Apply to FW",
            command=self._send_track_thresholds,
        ).pack(side="left", padx=(8, 0))

        ttk.Button(
            tr_row0, text="FW Pixel Dump",
            command=lambda: self.send_command("D"),
        ).pack(side="left", padx=(8, 0))

        ttk.Label(tr_row0, textvariable=self.track_var, foreground="red").pack(
            side="right", padx=(12, 0))

        # Row 1: HSV sliders
        tr_sliders = ttk.Frame(track_frame)
        tr_sliders.pack(fill="x", pady=(4, 0))

        hsv_sliders = [
            ("H lo", self.tr_h_lo, 0, 359),
            ("H hi", self.tr_h_hi, 0, 359),
            ("S min", self.tr_s_min, 0, 255),
            ("V min", self.tr_v_min, 0, 255),
            ("Min px", self.tr_min_px, 0, 2000),
        ]
        for idx, (label, var, lo, hi) in enumerate(hsv_sliders):
            c = idx * 3
            ttk.Label(tr_sliders, text=label).grid(row=0, column=c, sticky="e", padx=(8, 2))
            scale = ttk.Scale(
                tr_sliders, from_=lo, to=hi, orient="horizontal",
                variable=var, length=100,
                command=lambda _v, _var=var: self._on_track_param_changed(),
            )
            scale.grid(row=0, column=c + 1, sticky="ew", padx=2)
            val_lbl = ttk.Label(tr_sliders, text=str(var.get()), width=5)
            val_lbl.grid(row=0, column=c + 2, sticky="w", padx=(2, 4))
            var.trace_add("write", lambda *_a, v=var, l=val_lbl: l.configure(text=str(int(v.get()))))
            tr_sliders.columnconfigure(c + 1, weight=1)

        # Row 2: pixel inspector
        tr_row2 = ttk.Frame(track_frame)
        tr_row2.pack(fill="x", pady=(4, 0))
        ttk.Label(tr_row2, textvariable=self.pixel_var, foreground="blue").pack(side="left")

        # --- Camera Tuning sliders ---
        cam_frame = ttk.LabelFrame(container, text="Camera Tuning (OV7670)", padding=6)
        cam_frame.grid(row=4, column=0, sticky="ew", padx=8, pady=(0, 4))

        cam_sliders = [
            ("R Gain",     self.cam_r_gain,     0, 255, 0x02),
            ("B Gain",     self.cam_b_gain,     0, 255, 0x01),
            ("G Gain",     self.cam_g_gain,     0, 255, 0x6A),
            ("Saturation", self.cam_saturation, 0, 255, 0xC9),
            ("Brightness", self.cam_brightness, 0, 255, 0x55),
            ("Contrast",   self.cam_contrast,   0, 255, 0x56),
            ("MTX1 (R)",   self.cam_mtx1,       0, 255, 0x4F),
            ("MTX2 (G->R)",self.cam_mtx2,       0, 255, 0x50),
            ("MTX5 (G)",   self.cam_mtx5,       0, 255, 0x53),
            ("MTX6 (B)",   self.cam_mtx6,       0, 255, 0x54),
        ]

        self._cam_slider_regs: list[tuple[tk.IntVar, int]] = []
        cols_per_row = 3
        for idx, (label, var, lo, hi, reg) in enumerate(cam_sliders):
            r = idx // cols_per_row
            c = (idx % cols_per_row) * 3
            ttk.Label(cam_frame, text=label).grid(row=r, column=c, sticky="e", padx=(8, 2))
            scale = ttk.Scale(
                cam_frame, from_=lo, to=hi, orient="horizontal",
                variable=var, length=120,
                command=lambda val, rv=var, rg=reg: self._on_cam_slider(rv, rg),
            )
            scale.grid(row=r, column=c + 1, sticky="ew", padx=2)
            val_label = ttk.Label(cam_frame, text="0x00", width=6)
            val_label.grid(row=r, column=c + 2, sticky="w", padx=(2, 8))
            self._cam_slider_regs.append((var, reg))
            # Update label when var changes
            var.trace_add("write", lambda *_a, v=var, lbl=val_label: lbl.configure(
                text=f"0x{int(v.get()):02X}"))
            val_label.configure(text=f"0x{var.get():02X}")

        # Make slider columns expand
        for ci in range(cols_per_row):
            cam_frame.columnconfigure(ci * 3 + 1, weight=1)

        # --- Bottom: status + window snapshot + log ---
        status_bar = ttk.Frame(container, padding=(8, 0, 8, 8))
        status_bar.grid(row=5, column=0, sticky="nsew")
        status_bar.columnconfigure(0, weight=1)

        ttk.Label(status_bar, textvariable=self.frame_var).grid(row=0, column=0, sticky="w", pady=(0, 2))

        snapshot_frame = ttk.LabelFrame(status_bar, text="Latest Window / Freeze Snapshot", padding=4)
        snapshot_frame.grid(row=1, column=0, columnspan=2, sticky="ew", pady=(0, 6))
        snapshot_frame.columnconfigure(0, weight=1)
        self.snapshot_text = tk.Text(snapshot_frame, wrap="word", height=6)
        self.snapshot_text.grid(row=0, column=0, sticky="nsew")
        snap_scroll = ttk.Scrollbar(snapshot_frame, orient="vertical", command=self.snapshot_text.yview)
        snap_scroll.grid(row=0, column=1, sticky="ns")
        self.snapshot_text.configure(yscrollcommand=snap_scroll.set)
        self.snapshot_text.insert("end", "No window snapshot yet.\n")
        self.snapshot_text.configure(state="disabled")

        # Log header with clear button
        log_header = ttk.Frame(status_bar)
        log_header.grid(row=2, column=0, columnspan=2, sticky="ew")
        ttk.Label(log_header, text="UART Log").pack(side="left")
        ttk.Button(log_header, text="Clear", width=6, command=self._clear_log).pack(side="right")

        self.log_text = tk.Text(status_bar, wrap="word", height=6)
        self.log_text.grid(row=3, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(status_bar, orient="vertical", command=self.log_text.yview)
        scroll.grid(row=3, column=1, sticky="ns")
        self.log_text.configure(yscrollcommand=scroll.set)
        status_bar.rowconfigure(3, weight=1)

    def _reflow_buttons(self, event=None):
        """Re-layout command buttons into rows that fit the available width."""
        container = self._btn_flow_container
        avail_w = container.winfo_width()
        if avail_w <= 1:
            return

        inner = self._btn_flow_inner
        # Unpack all buttons first
        for btn in self._cmd_buttons:
            btn.grid_forget()

        # Measure button widths (update idletasks to get accurate reqwidth)
        inner.update_idletasks()
        pad_x = 4
        row = 0
        col = 0
        row_width = 0
        for btn in self._cmd_buttons:
            btn_w = btn.winfo_reqwidth() + pad_x * 2
            if col > 0 and row_width + btn_w > avail_w:
                row += 1
                col = 0
                row_width = 0
            btn.grid(row=row, column=col, padx=pad_x, pady=2, sticky="w")
            row_width += btn_w
            col += 1

    # --- Scroll event handlers ---
    def _on_inner_configure(self, _event=None):
        """Update scroll region when inner frame size changes."""
        self._canvas.configure(scrollregion=self._canvas.bbox("all"))

    def _on_canvas_configure(self, event=None):
        """Stretch inner frame width to fill canvas viewport."""
        if event:
            self._canvas.itemconfigure(self._canvas_win, width=event.width)

    def _on_mousewheel(self, event):
        """Scroll canvas with mouse wheel."""
        self._canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

    def refresh_ports(self):
        ports = list_ports()
        self.port_combo["values"] = ports
        if self.port_var.get() not in ports:
            self.port_var.set(default_port())

    def connect(self):
        if self.worker is not None:
            self.append_log("[HOST] Already connected.\n")
            return

        port = self.port_var.get().strip()
        if not port:
            self.status_var.set("No COM port selected")
            return

        self.worker = SerialWorker(port, BAUD, self.events)
        self.worker.start()

    def disconnect(self):
        self.stop_live_capture(silent=True)
        self.finish_auto_sweep("cancelled")
        worker = self.worker
        self.worker = None
        if worker is not None:
            worker.stop()

    def toggle_live_capture(self):
        if self.live_active:
            self.stop_live_capture()
        else:
            self.start_live_capture()

    def start_live_capture(self):
        if self.worker is None:
            self.append_log("[HOST] Connect a COM port first.\n")
            return
        if self.auto_active:
            self.append_log("[HOST] Stop auto sweep before starting live grab.\n")
            return
        self.live_active = True
        self.live_waiting_frame = False
        self.live_button.configure(text="Stop Live Grab")
        self.status_var.set("Live grab running")
        self._schedule_live_capture(50)

    def stop_live_capture(self, silent: bool = False):
        self.live_active = False
        self.live_waiting_frame = False
        if self.live_after_id is not None:
            self.root.after_cancel(self.live_after_id)
            self.live_after_id = None
        if hasattr(self, "live_button"):
            self.live_button.configure(text="Start Live Grab")
        if not silent:
            self.status_var.set("Live grab stopped")

    def _schedule_live_capture(self, delay_ms: int | None = None):
        if not self.live_active or self.worker is None or self.auto_active:
            return
        if self.live_after_id is not None:
            self.root.after_cancel(self.live_after_id)
        if delay_ms is None:
            try:
                delay_ms = int(self.live_interval_var.get())
            except Exception:
                delay_ms = LIVE_DEFAULT_INTERVAL_MS
        delay_ms = max(250, min(10000, delay_ms))
        self.live_after_id = self.root.after(delay_ms, self._live_capture_tick)

    def _live_capture_tick(self):
        self.live_after_id = None
        if not self.live_active or self.worker is None or self.auto_active or self.live_waiting_frame:
            return
        if self._send_command("u", echo=False):
            self.live_waiting_frame = True

    def send_command(self, cmd: str):
        if self.auto_active:
            self.append_log("[HOST] Auto sweep is running. Wait for it to finish.\n")
            return
        self._send_command(cmd, echo=True)

    def _send_command(self, cmd: str, echo: bool) -> bool:
        worker = self.worker
        if worker is None:
            self.append_log("[HOST] Connect a COM port first.\n")
            return False
        try:
            worker.send_command(cmd)
            if echo:
                self.append_log(f"[HOST] Sent '{cmd}'\n")
            return True
        except Exception as exc:
            self.append_log(f"[HOST] Send failed: {exc}\n")
            self.status_var.set("Send failed")
            return False

    def start_auto_sweep(self):
        if self.worker is None:
            self.append_log("[HOST] Connect a COM port first.\n")
            return
        if self.auto_active:
            self.append_log("[HOST] Auto sweep already running.\n")
            return
        if self.live_active:
            self.stop_live_capture(silent=True)
            self.append_log("[HOST] Live grab paused for auto sweep.\n")

        try:
            max_shift = int(self.auto_steps_var.get())
        except Exception:
            max_shift = 4
        max_shift = max(0, min(16, max_shift))
        self.auto_steps_var.set(max_shift)

        run_name = time.strftime("%Y%m%d-%H%M%S")
        self.auto_run_dir = Path.cwd() / "auto_sweeps" / run_name
        self.auto_run_dir.mkdir(parents=True, exist_ok=True)
        self.auto_active = True
        self.auto_pending_shift = None
        self.auto_max_shift = max_shift
        self.auto_timeout_token = 0
        self.status_var.set(f"Auto sweep running (0..{max_shift})")
        self.frame_var.set(f"Auto sweep saving into {self.auto_run_dir}")
        self.append_log(
            f"[HOST] Auto sweep start: Color Bar + baseline z + dumps for shift 0..{max_shift}\n"
        )

        if not self._send_command("c", echo=True):
            self.finish_auto_sweep("failed to enter color bar")
            return
        self.root.after(AUTO_SETTLE_MS, self.auto_apply_baseline)

    def auto_apply_baseline(self):
        if not self.auto_active:
            return
        if not self._send_command("z", echo=True):
            self.finish_auto_sweep("failed to apply baseline")
            return
        self.root.after(AUTO_SETTLE_MS, lambda: self.request_auto_frame(0))

    def request_auto_frame(self, shift: int):
        if not self.auto_active:
            return
        self.auto_pending_shift = shift
        if not self._send_command("u", echo=True):
            self.finish_auto_sweep("failed to request frame")
            return
        self.auto_timeout_token += 1
        token = self.auto_timeout_token
        self.root.after(AUTO_FRAME_TIMEOUT_MS, lambda: self.check_auto_timeout(token, shift))

    def check_auto_timeout(self, token: int, shift: int):
        if not self.auto_active:
            return
        if token != self.auto_timeout_token:
            return
        if self.auto_pending_shift == shift:
            self.finish_auto_sweep(f"timeout waiting for frame at shift {shift}")

    def continue_auto_sweep(self, just_captured_shift: int):
        if not self.auto_active:
            return
        if just_captured_shift >= self.auto_max_shift:
            self.finish_auto_sweep("done")
            return
        if not self._send_command("]", echo=True):
            self.finish_auto_sweep("failed to increment PSHFT")
            return
        next_shift = just_captured_shift + 1
        self.root.after(AUTO_SETTLE_MS, lambda s=next_shift: self.request_auto_frame(s))

    def finish_auto_sweep(self, reason: str):
        if not self.auto_active and reason == "cancelled":
            return
        run_dir = self.auto_run_dir
        self.auto_active = False
        self.auto_pending_shift = None
        self.auto_timeout_token += 1
        self.auto_max_shift = 0
        self.auto_run_dir = None
        if reason == "cancelled":
            self.status_var.set("Disconnected")
            return
        # Reset PSHFT=0 so live preview is not garbled after sweep
        self._send_command("z", echo=True)
        if run_dir is not None:
            self.append_log(f"[HOST] Auto sweep {reason}. Saved under {run_dir}\n")
            self.status_var.set(f"Auto sweep {reason}")
            self.frame_var.set(f"Auto sweep output: {run_dir}")
        else:
            self.append_log(f"[HOST] Auto sweep {reason}.\n")
            self.status_var.set(f"Auto sweep {reason}")

    _TRACK_RE = re.compile(
        r"\[TRACK\]\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)\s+(\d+)"
        r"(?:\s+ang=(\d+))?(?:\s+pwm=(\d+))?"
    )

    def _clear_log(self):
        self.log_text.delete("1.0", "end")

    def append_log(self, text: str):
        self.log_text.insert("end", text)
        self.log_text.see("end")
        self._consume_log_tags(text)

    def _consume_log_tags(self, text: str):
        for line in text.splitlines():
            if line.startswith("[WIN]"):
                self._parse_window_snapshot(line)
            if line.startswith("[TRACK]"):
                self._parse_track(line)

    def _parse_window_snapshot(self, line: str):
        if line.startswith("[WIN] state="):
            self.latest_window_lines = [line]
        elif self.latest_window_lines:
            self.latest_window_lines.append(line)
        else:
            self.latest_window_lines = [line]

        self.snapshot_text.configure(state="normal")
        self.snapshot_text.delete("1.0", "end")
        self.snapshot_text.insert("end", "\n".join(self.latest_window_lines) + "\n")
        self.snapshot_text.configure(state="disabled")

    def _parse_track(self, line: str):
        if "none" in line:
            self.track_info = None
            self.track_var.set("Tracking: not found")
            self.refresh_preview_images()
            return
        m = self._TRACK_RE.search(line)
        if not m:
            return
        ang = int(m.group(8)) if m.group(8) else None
        pwm = int(m.group(9)) if m.group(9) else None
        self.track_info = {
            "cx": int(m.group(1)),
            "cy": int(m.group(2)),
            "count": int(m.group(3)),
            "min_x": int(m.group(4)),
            "min_y": int(m.group(5)),
            "max_x": int(m.group(6)),
            "max_y": int(m.group(7)),
            "angle": ang,
            "pwm": pwm,
        }
        t = self.track_info
        extra = ""
        if ang is not None:
            extra += f" {ang}deg"
        if pwm is not None:
            extra += f" {pwm}us"
        if not self.track_local.get():
            self.track_var.set(f"FW: ({t['cx']},{t['cy']}) n={t['count']}{extra}")
        self.refresh_preview_images()

    @staticmethod
    def _decode_pixel_be(pixel_data: bytes, x: int, y: int, width: int):
        """Decode one pixel from big-endian RGB565 raw data."""
        off = (y * width + x) * 2
        msb = pixel_data[off]
        lsb = pixel_data[off + 1]
        raw = (msb << 8) | lsb
        r5 = (raw >> 11) & 0x1F
        g6 = (raw >> 5) & 0x3F
        b5 = raw & 0x1F
        return raw, r5, g6, b5

    @staticmethod
    def _rgb565_to_rgb888(r5: int, g6: int, b5: int):
        return r5 * 255 // 31, g6 * 255 // 63, b5 * 255 // 31

    @staticmethod
    def _rgb_to_hsv(r: int, g: int, b: int):
        """Return (h [0-360], s [0-255], v [0-255])."""
        mx = max(r, g, b)
        mn = min(r, g, b)
        diff = mx - mn
        v = mx
        s = 0 if mx == 0 else (diff * 255) // mx
        if diff == 0:
            h = 0
        elif mx == r:
            h = (60 * (g - b) // diff) % 360
        elif mx == g:
            h = 60 * (b - r) // diff + 120
        else:
            h = 60 * (r - g) // diff + 240
        return h, s, v

    def _is_red_hsv(self, h: int, s: int, v: int) -> bool:
        h_lo = self.tr_h_lo.get()  # e.g. 340
        h_hi = self.tr_h_hi.get()  # e.g. 30
        s_min = self.tr_s_min.get()
        v_min = self.tr_v_min.get()
        if s < s_min or v < v_min:
            return False
        # Red hue wraps around 0: h_lo..360 OR 0..h_hi
        if h_lo > h_hi:
            return h >= h_lo or h <= h_hi
        else:
            return h_lo <= h <= h_hi

    def _on_preview_click(self, event):
        """Show RGB565 values for the clicked pixel (both byte orders)."""
        if self.latest_raw_frame is None:
            return
        w, h, data = self.latest_raw_frame

        # Map click position back to source pixel coords
        label = self.main_preview_label
        lw, lh = label.winfo_width(), label.winfo_height()
        # The image is centered in the label
        img_ref = self.image_refs.get("__main__")
        if img_ref is None:
            return
        iw, ih = img_ref.width(), img_ref.height()
        ox = (lw - iw) // 2
        oy = (lh - ih) // 2
        ix = event.x - ox
        iy = event.y - oy
        if ix < 0 or iy < 0 or ix >= iw or iy >= ih:
            return
        # Scale to source
        src_x = int(ix * w / iw)
        src_y = int(iy * h / ih)
        src_x = max(0, min(w - 1, src_x))
        src_y = max(0, min(h - 1, src_y))

        _, r5, g6, b5 = self._decode_pixel_be(data, src_x, src_y, w)
        r8, g8, b8 = self._rgb565_to_rgb888(r5, g6, b5)
        h, s, v = self._rgb_to_hsv(r8, g8, b8)
        is_red = self._is_red_hsv(h, s, v)
        self.pixel_var.set(
            f"({src_x},{src_y}) "
            f"RGB=({r8},{g8},{b8}) "
            f"HSV=({h},{s},{v}) "
            f"red={'YES' if is_red else 'no'}"
        )

    def _detect_red_local(self, pixel_data: bytes, width: int, height: int):
        """Run red pixel detection on the GUI side using HSV color space."""
        min_px = self.tr_min_px.get()

        sum_x = 0
        sum_y = 0
        count = 0
        lo_x, lo_y = width, height
        hi_x, hi_y = 0, 0
        build_mask = self.show_mask.get()
        mask = bytearray(width * height) if build_mask else None

        for y in range(height):
            off = y * width * 2
            for x in range(width):
                msb = pixel_data[off + x * 2]
                lsb = pixel_data[off + x * 2 + 1]
                px = (msb << 8) | lsb
                r5 = (px >> 11) & 0x1F
                g6 = (px >> 5) & 0x3F
                b5 = px & 0x1F

                r8, g8, b8 = r5 * 255 // 31, g6 * 255 // 63, b5 * 255 // 31
                h, s, v = self._rgb_to_hsv(r8, g8, b8)

                if not self._is_red_hsv(h, s, v):
                    continue

                if mask is not None:
                    mask[y * width + x] = 255

                count += 1
                sum_x += x
                sum_y += y
                if x < lo_x:
                    lo_x = x
                if x > hi_x:
                    hi_x = x
                if y < lo_y:
                    lo_y = y
                if y > hi_y:
                    hi_y = y

        if count >= min_px:
            self.track_info = {
                "cx": sum_x // count,
                "cy": sum_y // count,
                "count": count,
                "min_x": lo_x,
                "min_y": lo_y,
                "max_x": hi_x,
                "max_y": hi_y,
            }
            t = self.track_info
            self.track_var.set(f"GUI: ({t['cx']},{t['cy']}) n={count}")
        else:
            self.track_info = None
            self.track_var.set(f"GUI: not found (n={count})")

        self._red_mask = mask
        self._red_mask_size = (width, height)
        self.refresh_preview_images()

    def _on_track_param_changed(self):
        if self.track_local.get() and self.latest_raw_frame is not None:
            w, h, data = self.latest_raw_frame
            self._detect_red_local(data, w, h)

    def _send_track_thresholds(self):
        """Send current HSV threshold values to firmware via UART command."""
        h_lo = self.tr_h_lo.get()
        h_hi = self.tr_h_hi.get()
        s_min = self.tr_s_min.get()
        v_min = self.tr_v_min.get()
        n = self.tr_min_px.get()
        cmd = f"T{h_lo},{h_hi},{s_min},{v_min},{n}\n"
        self._send_command(cmd, echo=True)
        self.append_log(f"[HOST] Sent HSV thresholds: H=[{h_lo}..{h_hi}] S>={s_min} V>={v_min} min_px={n}\n")

    def _on_cam_slider(self, var: tk.IntVar, reg: int):
        """Send OV7670 register write when a camera tuning slider changes (throttled)."""
        # Cancel any pending send for the same register
        pending_id = getattr(self, '_cam_slider_pending', {}).get(reg)
        if pending_id is not None:
            self.root.after_cancel(pending_id)
        if not hasattr(self, '_cam_slider_pending'):
            self._cam_slider_pending = {}
        # Delay 500ms so rapid dragging doesn't flood UART
        self._cam_slider_pending[reg] = self.root.after(
            500, lambda: self._cam_slider_send(var, reg)
        )

    def _cam_slider_send(self, var: tk.IntVar, reg: int):
        val = int(var.get())
        cmd = f"W{reg:02X},{val:02X}\n"
        self._send_command(cmd, echo=False)
        self._cam_slider_pending.pop(reg, None)

    def _draw_tracking_overlay(self, img: Image.Image, src_w: int, src_h: int) -> Image.Image:
        """Draw tracking bounding box + crosshair on a display-sized image."""
        t = self.track_info
        if t is None:
            return img
        img = img.copy()
        draw = ImageDraw.Draw(img)
        sx = img.width / src_w
        sy = img.height / src_h

        # Bounding box
        x0 = int(t["min_x"] * sx)
        y0 = int(t["min_y"] * sy)
        x1 = int((t["max_x"] + 1) * sx)
        y1 = int((t["max_y"] + 1) * sy)
        draw.rectangle([x0, y0, x1, y1], outline="lime", width=2)

        # Crosshair
        cx = int(t["cx"] * sx)
        cy = int(t["cy"] * sy)
        arm = 8
        draw.line([(cx - arm, cy), (cx + arm, cy)], fill="cyan", width=2)
        draw.line([(cx, cy - arm), (cx, cy + arm)], fill="cyan", width=2)

        # Label
        label = f"({t['cx']},{t['cy']}) n={t['count']}"
        if t.get("angle") is not None:
            label += f" {t['angle']}deg"
        if t.get("pwm") is not None:
            label += f" {t['pwm']}us"
        draw.text((x0, max(0, y0 - 14)), label, fill="yellow")
        return img

    def _apply_red_mask(self, img: Image.Image, src_w: int, src_h: int) -> Image.Image:
        """Overlay red mask on the image: red pixels bright red, rest dimmed."""
        mask_data = getattr(self, "_red_mask", None)
        if mask_data is None or not self.show_mask.get():
            return img
        mw, mh = getattr(self, "_red_mask_size", (0, 0))
        if mw != src_w or mh != src_h:
            return img

        mask_img = Image.frombytes("L", (mw, mh), bytes(mask_data))
        mask_img = mask_img.resize(img.size, RESAMPLING.NEAREST)

        img = img.copy()
        # Dim non-red pixels, highlight red pixels
        dimmed = img.point(lambda p: p // 3)
        red_overlay = Image.new("RGB", img.size, (255, 0, 0))
        # Blend: where mask=255 show red overlay, else show dimmed
        img = Image.composite(red_overlay, dimmed, mask_img)
        return img

    def refresh_preview_images(self):
        if not self.latest_variant_images:
            return

        main_title = self.main_variant_var.get()
        if main_title not in self.latest_variant_images:
            main_title = "Normal (BE)"

        src = self.latest_variant_images[main_title]
        main_img = fit_image(src, MAIN_PREVIEW_MAX, RESAMPLING.BICUBIC)
        main_img = self._apply_red_mask(main_img, src.width, src.height)
        main_img = self._draw_tracking_overlay(main_img, src.width, src.height)
        self.image_refs["__main__"] = ImageTk.PhotoImage(main_img)
        self.main_preview_label.configure(image=self.image_refs["__main__"])

        for title, img in self.latest_variant_images.items():
            thumb = fit_image(img, THUMB_PREVIEW_MAX, RESAMPLING.BILINEAR)
            self.image_refs[title] = ImageTk.PhotoImage(thumb)
            self.preview_labels[title].configure(image=self.image_refs[title])

    def update_frame(self, width: int, height: int, pixel_data: bytes):
        self.latest_raw_frame = (width, height, pixel_data)

        variants = frame_variants(pixel_data)
        decoded_images: dict[str, Image.Image] = {}

        out_dir = Path.cwd()
        for title, raw in variants.items():
            img = rgb565_to_image(raw, width, height)
            decoded_images[title] = img
            img.save(out_dir / FILE_MAP[title])
        self.latest_variant_images = decoded_images

        if self.track_local.get():
            self._detect_red_local(pixel_data, width, height)
        else:
            self.refresh_preview_images()

        self.frame_var.set(
            "Frame "
            f"{width}x{height} saved as frame_normal.png / frame_swapped.png / "
            "frame_wordswapped.png / frame_word_byteswapped.png"
        )

        if self.auto_active and self.auto_pending_shift is not None and self.auto_run_dir is not None:
            prefix = f"shift_{self.auto_pending_shift:02d}"
            for title, img in decoded_images.items():
                img.save(self.auto_run_dir / f"{prefix}_{FILE_MAP[title]}")

    def poll_events(self):
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "log":
                    self.append_log(payload)
                elif kind == "status":
                    self.status_var.set(payload)
                    if payload == "Disconnected":
                        self.stop_live_capture(silent=True)
                        self.finish_auto_sweep("cancelled")
                        self.worker = None
                elif kind == "error":
                    self.status_var.set(payload)
                    self.append_log(f"[HOST] {payload}\n")
                    self.stop_live_capture(silent=True)
                    self.finish_auto_sweep("failed")
                    self.worker = None
                elif kind == "frame":
                    width, height, _bpp, pixel_data = payload
                    self.update_frame(width, height, pixel_data)
                    self.append_log(f"[HOST] Received frame {width}x{height}\n")
                    self.live_waiting_frame = False
                    if self.auto_active and self.auto_pending_shift is not None:
                        captured_shift = self.auto_pending_shift
                        self.auto_pending_shift = None
                        self.root.after(
                            AUTO_SETTLE_MS, lambda s=captured_shift: self.continue_auto_sweep(s)
                        )
                    elif self.live_active:
                        self._schedule_live_capture()
        except queue.Empty:
            pass

        self.root.after(50, self.poll_events)

    def on_close(self):
        self.disconnect()
        self.root.after(100, self.root.destroy)


def main():
    root = tk.Tk()
    style = ttk.Style(root)
    if "vista" in style.theme_names():
        style.theme_use("vista")
    DebugGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

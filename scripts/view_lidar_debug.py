#!/usr/bin/env python3
"""Desktop point cloud + 187-cell viewer. Reads a GOS SSH stream or a saved JSONL.

This UI has no motion controls or ROS publishers. A remote collector exists only
for this bounded session and does not start or restart any robot service.
"""
import argparse
import getpass
import json
import math
import os
from pathlib import Path
import shlex
import signal
import subprocess
import sys
import threading
import time

import numpy as np


def cell_state(terrain, elapsed, limits):
    """Expire measurements, keeping raw sender diagnostics separate from display state."""
    a = {k: np.asarray(terrain[k], dtype=float) for k in ("height", "validity", "confidence", "variance", "age")}
    if any(v.shape != (187,) or not np.isfinite(v).all() for v in a.values()):
        raise ValueError("terrain needs five finite 187-cell arrays")
    if not np.isin(a["validity"], (0, 1)).all():
        raise ValueError("invalid validity mask")
    header_age = float(terrain["header_age_s"]) + elapsed
    if not np.isfinite(header_age):
        raise ValueError("nonfinite terrain timestamp")
    future = header_age < -limits["future_tolerance_s"]
    fresh = not future and header_age <= limits["terrain_frame_age_s"]
    ages = a["age"] + max(0., header_age)
    known = (a["validity"] == 1) & (ages <= limits["max_cell_age_s"]) & fresh
    return a, known, ages, fresh, future


def color(value, lo, hi):
    from PyQt5.QtGui import QColor
    t = min(1., max(0., (value - lo) / max(1e-9, hi - lo)))
    return QColor.fromHsvF((1 - t) * .66, .72, .95)


def displayed_cell_state(terrain, elapsed, limits, hz, strict):
    a, known, ages, fresh, future = cell_state(terrain, elapsed if strict else 0., limits)
    disconnected = elapsed + terrain.get("silence_s", 0.) > max(1., 2 / hz)
    if disconnected:
        known = np.zeros(187, dtype=bool)
    return a, known, ages, fresh, future, disconnected


def origin_offset(payload, selection):
    if selection == "body":
        offset = payload.get("body_origin_m", [0., 0., 0.])
    else:
        if "front_lidar_origin_m" not in payload:
            raise ValueError(payload.get("front_origin_error", "缺少前雷达外参/对应时刻位姿"))
        offset = payload["front_lidar_origin_m"]
    offset = np.asarray(offset, dtype=float)
    if offset.shape != (3,) or not np.isfinite(offset).all():
        raise ValueError("显示原点坐标异常")
    return offset


TERRAIN_SOURCES = ("default", "baseline", "candidate", "published")
TERRAIN_LABELS = ("当前数据源", "原版旁路", "候选旁路（未接入模型）", "板端实际发布")
LAYERS = ("height", "ground_z", "validity", "confidence", "age", "variance", "height_delta")


def selected_terrain(data, selection):
    if selection not in TERRAIN_SOURCES:
        raise ValueError("unknown terrain source")
    key = "terrain" if selection == "default" else "terrain_" + selection
    return data.get("streams", {}).get(key) if data else None


def height_difference(data, selection, elapsed, hz, strict):
    """Compare only explicitly supplied, matching time/geometry observation pairs."""
    current, baseline = selected_terrain(data, selection), selected_terrain(data, "baseline")
    if current is None or baseline is None:
        raise ValueError("缺少原版或当前来源，无法比较")
    identity_keys = ("frame", "profile", "geometry_hash", "sequence")
    if (any(k not in current or k not in baseline or current[k] != baseline[k] for k in identity_keys) or
            "stamp" not in current or "stamp" not in baseline or
            not np.isfinite([current["stamp"], baseline["stamp"]]).all() or
            abs(current["stamp"] - baseline["stamp"]) > 1e-6):
        raise ValueError("两个来源的时间或坐标不一致，无法比较")
    a, known, *_ = displayed_cell_state(current, elapsed, data["limits"], hz, strict)
    b, base_known, *_ = displayed_cell_state(baseline, elapsed, data["limits"], hz, strict)
    return 1000.*(a["height"]-b["height"]), known & base_known


def grid_layout(width, height, offset):
    # Keep original cells and values; recenter the viewport without rebinning.
    half_x = max(abs(-.85 - offset[0]), abs(.85 - offset[0]))
    half_y = max(abs(-.55 - offset[1]), abs(.55 - offset[1]))
    scale = min((width - 110) / (2 * half_x), (height - 180) / (2 * half_y))
    return np.array([width / 2, 83 + (height - 180) / 2]), scale


def grid_pixel(xy, center, scale, offset):
    return center + (np.asarray(xy) - offset[:2]) * [scale, -scale]


def grid_cell(pixel, center, scale, offset):
    xy = (np.asarray(pixel) - center) / [scale, -scale] + offset[:2]
    ix, iy = np.floor((xy + [.85, .55]) / .1).astype(int)
    return (int(ix), int(iy)) if 0 <= ix < 17 and 0 <= iy < 11 else None


class Stream:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest = None
        self.received = 0.
        self.state = "等待数据"
        self.process = None
        self.stopped = threading.Event()
        self.frozen = None
        self.stderr_tail = []
        self.ssh_clients = []
        self.remote_channel = None

    def read_errors(self, handle):
        for line in handle:
            with self.lock:
                self.stderr_tail = (self.stderr_tail + [line.strip()])[-4:]

    def read(self, handle, replay=False):
        try:
            for line in handle:
                if self.stopped.is_set():
                    break
                if len(line) > 4_000_000:
                    raise ValueError("debug record exceeds size budget")
                packet = json.loads(line)
                if packet.get("schema") != "m20_lidar_debug_v1":
                    raise ValueError("unsupported stream schema")
                with self.lock:
                    self.latest, self.received = packet, time.monotonic()
                    self.state = "历史记录（回放）" if replay else "实时数据"
                if replay and self.stopped.wait(.5):
                    break
            with self.lock:
                self.state = "回放结束" if replay else "连接已结束；保留最后一帧供检查"
        except Exception as exc:
            with self.lock:
                self.state = "数据错误：" + str(exc)

    def snapshot(self):
        with self.lock:
            if self.frozen is not None:
                return self.frozen
            elapsed = 0. if "回放" in self.state else max(0., time.monotonic() - self.received)
            state = self.state
            if self.process is not None and self.process.poll() not in (None, 0):
                state += " | SSH/采集器退出 " + str(self.process.returncode) + " | " + " ".join(self.stderr_tail)
            return self.latest, elapsed, state

    def close(self):
        self.stopped.set()
        if self.remote_channel is not None:
            self.remote_channel.close()
        for client in reversed(self.ssh_clients):
            client.close()
        self.ssh_clients.clear()
        if self.process is not None and self.process.poll() is None:
            os.killpg(self.process.pid, signal.SIGTERM)
            try:
                self.process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(self.process.pid, signal.SIGKILL)
                self.process.wait(timeout=3)


def remote_command(args):
    remote_args = ["--config", args.config, "--seconds", str(args.seconds),
                   "--hz", str(args.hz), "--max-points", str(args.max_points)]
    remote = ("cd " + shlex.quote(args.release) +
              " && source scripts/board_env.sh >/dev/null && exec timeout --signal=INT " +
              str(math.ceil(args.seconds + 5)) +
              " nice -n 15 /usr/bin/python3 -u - " + shlex.join(remote_args))
    return "bash -c " + shlex.quote(remote)


def ssh_command(args):
    # Shell quoting applies only to remote paths; SSH identity/options remain argv.
    cmd = ["ssh", "-T", "-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=yes",
           "-o", "ConnectTimeout=8", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2"]
    if args.jump:
        cmd.extend(["-J", args.jump])
    cmd.extend([args.ssh_target, remote_command(args)])
    return cmd


def ssh_address(target):
    """Resolve the explicit host/alias; never use a guessed robot identity."""
    import paramiko
    explicit_user, sep, host = target.rpartition("@")
    if not sep:
        host = target
    config = paramiko.SSHConfig()
    path = Path.home() / ".ssh/config"
    if path.exists():
        with path.open() as handle:
            config.parse(handle)
    entry = config.lookup(host)
    return entry.get("hostname", host), explicit_user or entry.get("user", getpass.getuser()), int(entry.get("port", 22))


def password_stream(args, stream, script):
    """Authenticate each hop independently; nested sshpass can consume the wrong prompt."""
    import paramiko
    password = os.environ.get(args.password_env)
    if not password:
        raise ValueError("selected credential environment variable is empty")

    def connect(target, sock=None):
        host, user, port = ssh_address(target)
        client = paramiko.SSHClient()
        client.load_system_host_keys()
        client.set_missing_host_key_policy(paramiko.RejectPolicy())
        stream.ssh_clients.append(client)
        client.connect(host, port=port, username=user, password=password, sock=sock,
                       allow_agent=False, look_for_keys=False, timeout=8, auth_timeout=8, banner_timeout=8)
        client.get_transport().set_keepalive(5)
        return client

    sock = None
    if args.jump:
        jump = connect(args.jump)
        host, user, port = ssh_address(args.ssh_target)
        sock = jump.get_transport().open_channel("direct-tcpip", (host, port), ("127.0.0.1", 0), timeout=8)
    client = connect(args.ssh_target, sock)
    stdin, stdout, stderr = client.exec_command(remote_command(args))
    stream.remote_channel = stdout.channel
    threading.Thread(target=stream.read_errors, args=(stderr,), daemon=True).start()
    stdin.write(script)
    stdin.flush()
    stdin.channel.shutdown_write()
    return stdout


def run_gui(args, stream):
    from PyQt5 import QtCore, QtGui, QtWidgets

    class CloudView(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.yaw, self.pitch, self.zoom = -.6, .65, 80.
            self.last = None
            self.setMinimumSize(480, 440)

        def mousePressEvent(self, e):
            self.last = e.pos()

        def mouseMoveEvent(self, e):
            if self.last is not None and e.buttons() & QtCore.Qt.LeftButton:
                d = e.pos() - self.last
                self.yaw += d.x() * .01
                self.pitch = np.clip(self.pitch + d.y() * .01, -.1, 1.5)
                self.last = e.pos()
                self.update()

        def wheelEvent(self, e):
            self.zoom = np.clip(self.zoom * math.exp(e.angleDelta().y() / 1200), 15., 600.)
            self.update()

        def paintEvent(self, event):
            p = QtGui.QPainter(self)
            p.fillRect(self.rect(), QtGui.QColor("#111a27"))
            p.setPen(QtGui.QColor("#edf3fc"))
            data, elapsed, state = stream.snapshot()
            key = "front" if window.cloud_mode.currentIndex() else "raw"
            cloud = data.get("streams", {}).get(key) if data else None
            p.drawText(18, 28, "前向扇形点云" if key == "front" else "完整雷达点云")
            if cloud is None:
                p.drawText(18, 60, "尚未收到真实点云；请检查 GOS 感知是否正在运行")
                return
            points = np.asarray(cloud["xyz"], dtype=float).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            try:
                offset = origin_offset(cloud, window.origin_selection())
            except ValueError as exc:
                p.drawText(18, 60, "无法切换原点：" + str(exc))
                return
            points = points - offset
            radius = window.range_box.value()
            points = points[np.linalg.norm(points[:, :2], axis=1) <= radius]
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            cp, sp = math.cos(self.pitch), math.sin(self.pitch)
            basis = np.array([[c, -s, 0], [s * sp, c * sp, cp], [s * cp, c * cp, -sp]])

            def project(xyz):
                v = np.asarray(xyz) @ basis.T
                return np.column_stack((self.width() / 2 + v[:, 0] * self.zoom,
                                        self.height() / 2 - v[:, 1] * self.zoom))

            xy = project(points)
            for i in range(-int(radius), int(radius) + 1):
                for line in ([[-radius, i, 0], [radius, i, 0]], [[i, -radius, 0], [i, radius, 0]]):
                    q = project(line)
                    p.setPen(QtGui.QColor("#26354a"))
                    p.drawLine(QtCore.QPointF(*q[0]), QtCore.QPointF(*q[1]))
            for axis, name, hue in (([1, 0, 0], "X", "#fb8176"), ([0, 1, 0], "Y", "#76dfad"), ([0, 0, 1], "Z", "#81b5ff")):
                q = project([[0, 0, 0], axis])
                p.setPen(QtGui.QPen(QtGui.QColor(hue), 2))
                p.drawLine(QtCore.QPointF(*q[0]), QtCore.QPointF(*q[1]))
                p.drawText(QtCore.QPointF(*q[1]), name)
            body = np.asarray(cloud.get("body_origin_m", [0., 0., 0.])) - offset
            if window.origin_selection() == "front":
                q = project([body])[0]
                p.setPen(QtGui.QColor("#f8d875"))
                p.drawEllipse(QtCore.QPointF(*q), 5, 5)
                p.drawText(QtCore.QPointF(*q) + QtCore.QPointF(8, 0), "机身")
            for point, q in zip(points, xy):
                if 0 < q[0] < self.width() and 75 < q[1] < self.height() - 45:
                    p.setPen(QtGui.QPen(color(point[2], -1., .5), 3))
                    p.drawPoint(QtCore.QPointF(*q))
            p.setPen(QtGui.QColor("#d5dfed"))
            age = cloud["header_age_s"] + elapsed
            # Scan header age includes scan duration. This is a display warning only.
            limit = data["limits"]["cloud_frame_age_s"] + data["limits"]["scan_duration_s"]
            stale = cloud["silence_s"] + elapsed > limit or age > limit or age < -data["limits"]["future_tolerance_s"]
            p.drawText(18, 53, f"原点：{window.origin_label()} | 原始 {cloud['source_points']} 点 / 显示 {len(points)} 点")
            p.drawText(18, 75, f"源 frame: {cloud['frame']} · 保持原坐标轴方向")
            p.drawText(18, self.height() - 25, f"{'过期/断流' if stale else '最近收到'} · 扫描 header 年龄 {age * 1000:.0f} ms · 拖动旋转，滚轮缩放")

    class GridView(QtWidgets.QWidget):
        def __init__(self):
            super().__init__()
            self.setMinimumSize(640, 440)
            self.setMouseTracking(True)
            self.grid_rect = QtCore.QRectF()
            self.layout_state = None

        def paintEvent(self, event):
            p = QtGui.QPainter(self)
            p.fillRect(self.rect(), QtGui.QColor("#172232"))
            self.layout_state = None
            p.setPen(QtGui.QColor("#edf3fc"))
            data, elapsed, state = stream.snapshot()
            terrain = selected_terrain(data, window.terrain_selection())
            p.drawText(18, 28, window.terrain_mode.currentText() + " · 11 × 17 格 · X 向前 / Y 向左")
            if not terrain:
                p.drawText(18, 60, "尚未收到所选来源；不会用其它来源替代")
                return
            try:
                strict = window.strict_age.isChecked()
                a, known, ages, fresh, future, disconnected = displayed_cell_state(terrain, elapsed, data["limits"], args.hz, strict)
                offset = origin_offset(terrain, window.origin_selection())
            except (KeyError, ValueError) as exc:
                p.drawText(18, 60, "高程协议错误：" + str(exc))
                return
            k = window.grid_mode.currentIndex()
            key = LAYERS[k]
            if key == "height_delta":
                try:
                    values, comparable = height_difference(data, window.terrain_selection(), elapsed, args.hz, strict)
                    known &= comparable
                except ValueError as exc:
                    p.drawText(18, 60, "差值不可用：" + str(exc))
                    return
            else:
                values = -.5 - a["height"] - offset[2] if key == "ground_z" else ages if key == "age" else a[key]
            if key == "ground_z":
                known = known & (np.abs(a["height"]) < 1.)
            lo, hi = {"height": (-.25, .25), "validity": (0., 1.), "confidence": (0., 1.),
                      "ground_z": (-.7, .3),
                      "age": (0., data["limits"]["max_cell_age_s"]), "variance": (0., .01), "height_delta": (-10., 10.)}[key]
            if key in ("height", "ground_z") and known.any():
                lo, hi = float(values[known].min()), float(values[known].max())
                if hi - lo < .02:
                    lo, hi = lo - .01, hi + .01
            center, scale = grid_layout(self.width(), self.height(), offset)
            step = scale * .1
            corner = grid_pixel([-.85, .55], center, scale, offset)
            self.grid_rect = QtCore.QRectF(*corner, 17 * step, 11 * step)
            self.layout_state = (center, scale, offset)
            for iy in range(11):
                for ix in range(17):
                    idx = iy * 17 + ix
                    pixel = grid_pixel([-.85 + ix * .1, -.45 + iy * .1], center, scale, offset)
                    r = QtCore.QRectF(*pixel, step - 1, step - 1)
                    p.fillRect(r, color(values[idx], lo, hi) if known[idx] else QtGui.QColor("#596271"))
                    if not known[idx]:
                        p.setPen(QtGui.QColor("#aeb8c5"))
                        p.drawText(r, QtCore.Qt.AlignCenter, "?")
            p.setPen(QtGui.QColor("white"))
            mark = QtCore.QPointF(*center)
            p.drawLine(mark + QtCore.QPointF(-6, 0), mark + QtCore.QPointF(6, 0))
            p.drawLine(mark + QtCore.QPointF(0, -6), mark + QtCore.QPointF(0, 6))
            p.drawText(mark + QtCore.QPointF(8, -7), window.origin_label() + " (0,0)")
            if window.origin_selection() == "front":
                body = QtCore.QPointF(*grid_pixel([0., 0.], center, scale, offset))
                p.setPen(QtGui.QColor("#f8d875"))
                p.drawEllipse(body, 5, 5)
                p.drawText(body + QtCore.QPointF(8, 14), "机身")
            p.setPen(QtGui.QColor("white"))
            p.drawText(18, self.height() - 63, f"X 格中心 [{-.8-offset[0]:+.3f}, {.8-offset[0]:+.3f}] m · 原点：{window.origin_label()} · +X 向前")
            mode = "共同可比" if key == "height_delta" else "当前时效有效" if strict else "采样时有效"
            p.drawText(18, 62, f"源 frame: {terrain['frame']} | 源数据有效 {int(a['validity'].sum())}/187 | {mode} {int(known.sum())}/187")
            status = "数据断流" if disconnected else "未来时间异常" if future else ("当前时效通过" if strict else "显示采样快照") if fresh else "本帧过期"
            p.drawText(18, self.height() - 38, f"{status} · sequence {terrain['sequence']} · 色标 {key}：{lo:g} 蓝 → {hi:g} 红")
            note = "Δh = 当前来源 h − 原版 h，单位 mm；正值表示估计地面更低。灰色 ? = 不可比。" if key == "height_delta" else "Z 为相对所选原点的地面高度（由未饱和 h 换算）；策略 h 保持原定义。灰色 ? = 未知。"
            p.drawText(18, self.height() - 14, note)

        def mouseMoveEvent(self, e):
            if self.layout_state is None or not self.grid_rect.contains(e.pos()):
                return
            center, scale, offset = self.layout_state
            cell = grid_cell([e.x(), e.y()], center, scale, offset)
            if cell is None:
                return
            ix, iy = cell
            idx = iy * 17 + ix
            data, elapsed, state = stream.snapshot()
            t = selected_terrain(data, window.terrain_selection())
            if t:
                strict = window.strict_age.isChecked()
                a, known, ages, fresh, future, disconnected = displayed_cell_state(t, elapsed, data["limits"], args.hz, strict)
                z = f"{-.5-a['height'][idx]-offset[2]:+.4f} m" if known[idx] and abs(a['height'][idx]) < 1 else "未知/已裁剪"
                age_text = f"{ages[idx]*1000:.0f} ms" if a['age'][idx] >= 0 else "未知"
                difference = ""
                if window.grid_mode.currentIndex() == LAYERS.index("height_delta"):
                    try:
                        delta, comparable = height_difference(data, window.terrain_selection(), elapsed, args.hz, strict)
                        difference = f" | Δh={delta[idx]:+.3f} mm" if comparable[idx] else " | Δh=不可比"
                    except ValueError:
                        difference = " | Δh=不可比"
                window.details.setText(f"帧 {t['sequence']} 格 {idx} | 原点：{window.origin_label()} | x={-.8 + ix*.1-offset[0]:+.3f} m, y={-.5 + iy*.1-offset[1]:+.3f} m | "
                    f"Z={z} | "
                    f"h={a['height'][idx]:+.4f} m | valid(发送)={a['validity'][idx]:.0f} | "
                    f"{'当前' if strict else '采样时'}显示={'有效' if known[idx] else '未知'} | c={a['confidence'][idx]:.3f} | "
                    f"age={age_text} | variance={a['variance'][idx]:.5f} m²" + difference)

    class Window(QtWidgets.QMainWindow):
        def __init__(self):
            super().__init__()
            self.setWindowTitle("M20 · 雷达点云 / 高程图 · 只读调试")
            self.resize(1420, 790)
            self.setStyleSheet("QWidget{background:#101927;color:#e4edf9;font-size:14px} QComboBox,QPushButton,QDoubleSpinBox{padding:5px;background:#24344b} QLabel{padding:5px}")
            body = QtWidgets.QWidget()
            self.setCentralWidget(body)
            layout = QtWidgets.QVBoxLayout(body)
            title = QtWidgets.QLabel("M20  雷达点云 → 高程观测")
            title.setStyleSheet("font-size:24px;font-weight:600")
            layout.addWidget(title)
            self.terrain_mode = QtWidgets.QComboBox()
            self.terrain_mode.setObjectName("terrain_mode")
            self.terrain_mode.addItems(TERRAIN_LABELS)
            self.terrain_mode.setCurrentIndex(TERRAIN_SOURCES.index(getattr(args, "terrain_source", "default")))
            layout.addWidget(self.terrain_mode)
            self.status = QtWidgets.QLabel("等待只读数据源")
            self.status.setWordWrap(True)
            layout.addWidget(self.status)
            toolbar = QtWidgets.QHBoxLayout()
            self.cloud_mode = QtWidgets.QComboBox()
            self.cloud_mode.addItems(["完整雷达点云", "实际送入 mapper 的前向点云"])
            self.cloud_mode.setCurrentIndex(int(args.cloud == "front"))
            self.grid_mode = QtWidgets.QComboBox()
            self.grid_mode.addItems(["模型高度 h (m)", "相对原点地面高度 Z (m)", "有效性", "置信度", "单格年龄 (s)", "方差 (m²)", "与原版高度差 Δh (mm)"])
            self.grid_mode.setCurrentIndex(LAYERS.index(getattr(args, "layer", "height")))
            self.origin_mode = QtWidgets.QComboBox()
            self.origin_mode.setObjectName("origin_mode")
            self.origin_mode.addItems(["原点：机身中心", "原点：前方雷达"])
            self.origin_mode.setCurrentIndex(int(args.origin == "front"))
            self.range_box = QtWidgets.QDoubleSpinBox()
            self.range_box.setRange(1, 30)
            self.range_box.setValue(5)
            self.range_box.setSuffix(" m 显示半径")
            self.strict_age = QtWidgets.QCheckBox("严格实时效期")
            toolbar.addWidget(self.cloud_mode)
            toolbar.addWidget(self.origin_mode)
            toolbar.addWidget(self.range_box)
            toolbar.addWidget(self.grid_mode)
            toolbar.addWidget(self.strict_age)
            snapshot = QtWidgets.QPushButton("保存当前数据和截图")
            snapshot.clicked.connect(self.save)
            toolbar.addWidget(snapshot)
            layout.addLayout(toolbar)
            views = QtWidgets.QHBoxLayout()
            self.cloud, self.grid = CloudView(), GridView()
            views.addWidget(self.cloud, 1)
            views.addWidget(self.grid, 1)
            layout.addLayout(views, 1)
            self.details = QtWidgets.QLabel("移动到高程格查看原始数值、坐标与年龄")
            self.details.setWordWrap(True)
            layout.addWidget(self.details)
            self.diagnostics = QtWidgets.QPlainTextEdit()
            self.diagnostics.setReadOnly(True)
            self.diagnostics.setMaximumHeight(110)
            layout.addWidget(self.diagnostics)
            note = QtWidgets.QLabel("原厂融合云已在机体系；高程图保持重力对齐。原点切换同时平移两栏，不重采样高程格。")
            layout.addWidget(note)
            self.timer = QtCore.QTimer(self)
            self.timer.timeout.connect(self.refresh)
            self.timer.start(200)
            self.origin_mode.currentIndexChanged.connect(self.refresh)
            self.terrain_mode.currentIndexChanged.connect(self.refresh)

        def terrain_selection(self):
            return TERRAIN_SOURCES[self.terrain_mode.currentIndex()]

        def origin_selection(self):
            return "front" if self.origin_mode.currentIndex() else "body"

        def origin_label(self):
            return "前方雷达" if self.origin_selection() == "front" else "机身中心"

        def refresh(self):
            data, elapsed, state = stream.snapshot()
            if data:
                identity = data["identity"]
                labels = data.get("terrain_source_labels", {})
                for index, key in enumerate(TERRAIN_SOURCES):
                    label = labels.get(key) if isinstance(labels, dict) else None
                    self.terrain_mode.setItemText(index, label[:80] if isinstance(label, str) and label else TERRAIN_LABELS[index])
                self.status.setText(f"{state} | {identity['board']} | serial={identity['serial']} | "
                    f"最后数据到达 {elapsed:.1f}s 前 | {args.hz:g} Hz 显示采样\n"
                    f"raw={data['topics']['raw']}  →  mapper={data['topics']['front']}  →  terrain={data['topics']['terrain']}\n"
                    f"诊断：{json.dumps(data.get('errors', {}), ensure_ascii=False)}")
                self.diagnostics.setPlainText(json.dumps({"received_counts": data.get("counts", {}),
                    "mapper_status": data.get("streams", {}).get("mapper_status", {}),
                    "native_status": data.get("streams", {}).get("native_status", {}),
                    "source_notes": data.get("source_notes", "")}, ensure_ascii=False))
            else:
                self.status.setText(state + "；尚未验证真实数据")
            self.cloud.update()
            self.grid.update()

        def save(self):
            data, elapsed, state = stream.snapshot()
            if data is None:
                self.details.setText("没有真实数据，未保存空截图")
                return
            dst = Path(args.output)
            dst.mkdir(parents=True, exist_ok=True)
            prefix = dst / (time.strftime("snapshot_%Y%m%d_%H%M%S") + f"_{time.time_ns()%1000000:06d}")
            result = {"schema": "m20_lidar_debug_snapshot_v1", "stream_state": state,
                      "display_elapsed_s": elapsed, "strict_age_display": self.strict_age.isChecked(),
                      "display_origin": self.origin_selection(), "display_layer": self.grid_mode.currentIndex(), "packet": data}
            result["display_terrain_source"] = self.terrain_selection()
            with stream.lock:
                previous_frozen = stream.frozen
                stream.frozen = data, elapsed, state + "（保存快照）"
            try:
                self.refresh()
                prefix.with_suffix(".json").write_text(json.dumps(result, ensure_ascii=False, allow_nan=False))
                self.grab().save(str(prefix.with_suffix(".png")))
            finally:
                with stream.lock:
                    stream.frozen = previous_frozen
            self.details.setText("已保存 " + str(prefix))

        def closeEvent(self, e):
            stream.close()
            e.accept()

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    window = Window()
    window.show()
    if args.screenshot:
        def capture():
            window.refresh()
            window.grab().save(args.screenshot)
            window.close()
        QtCore.QTimer.singleShot(1500, capture)
    if args.startup_snapshot:
        QtCore.QTimer.singleShot(5000, window.save)
    return app.exec_()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--ssh-target", help="GOS SSH host/alias; optionally reached using --jump")
    src.add_argument("--replay", help="saved collector JSONL, always labelled historical")
    p.add_argument("--jump", help="existing SSH jump alias; no passwords stored by this tool")
    p.add_argument("--password-env", help="optional credential environment variable; jump and GOS must share it")
    p.add_argument("--release", default="/home/user/m20_heightmap/current")
    p.add_argument("--config", default="runtime/heightmap_v1.local.json")
    p.add_argument("--cloud", choices=("raw", "front"), default="raw")
    p.add_argument("--origin", choices=("body", "front"), default="body", help="display origin; axes keep their existing directions")
    p.add_argument("--terrain-source", choices=TERRAIN_SOURCES, default="default", help="select an explicitly supplied terrain stream; absent sources stay unavailable")
    p.add_argument("--layer", choices=LAYERS, default="height", help="initial displayed layer; height_delta requires a matching baseline")
    p.add_argument("--seconds", type=float, default=1800)
    p.add_argument("--hz", type=float, default=2)
    p.add_argument("--max-points", type=int, default=3000)
    p.add_argument("--output", default="recordings/lidar_debug")
    p.add_argument("--screenshot", help=argparse.SUPPRESS)
    p.add_argument("--startup-snapshot", action="store_true", help="save actual received data + screenshot after five seconds")
    args = p.parse_args()
    if not 1 <= args.seconds <= 3600 or not .2 <= args.hz <= 5 or not 100 <= args.max_points <= 10000:
        p.error("bounded debug budget: seconds 1..3600, hz .2..5, points 100..10000")
    stream = Stream()
    if args.replay:
        handle = open(args.replay)
    else:
        env = dict(os.environ, OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
        script = Path(__file__).with_name("stream_lidar_debug.py").read_text()
        if args.password_env:
            try:
                handle = password_stream(args, stream, script)
            except Exception as exc:
                stream.close()
                p.error("SSH connection failed: " + str(exc))
        else:
            stream.process = subprocess.Popen(ssh_command(args), stdin=subprocess.PIPE,
                                          stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                          text=True, env=env, start_new_session=True)
            threading.Thread(target=stream.read_errors, args=(stream.process.stderr,), daemon=True).start()
            try:
                stream.process.stdin.write(script)
                stream.process.stdin.close()
            except BrokenPipeError:
                pass
            handle = stream.process.stdout
    worker = threading.Thread(target=stream.read, args=(handle, bool(args.replay)), daemon=True)
    worker.start()
    try:
        return run_gui(args, stream)
    finally:
        stream.close()


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""持续采集摄像头画面，只向控制循环提供最新帧及其采集时间。"""

import threading
import time

import cv2

import config as C


class LatestFrameCamera:
    """
    单独线程持续采集，只保留最新一帧。
    目的：避免主循环视觉计算导致 VideoCapture 缓冲越积越多。
    """

    def __init__(self):
        """准备相机、采集线程与受锁保护的最新画面存储，暂不打开设备。"""
        self.cap = None
        self.thread = None
        self.running = False

        self.lock = threading.Lock()
        self.frame = None
        self.frame_id = 0
        self.timestamp = 0.0

    def start(self):
        """按标定配置打开相机、预热并启动采集线程，等到首帧后返回。"""
        if self.running:
            return self

        self.cap = cv2.VideoCapture(C.CAMERA_INDEX)

        if C.CAMERA_USE_MJPG:
            fourcc = cv2.VideoWriter_fourcc(*"MJPG")
            self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, C.CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, C.CAMERA_HEIGHT)
        self.cap.set(cv2.CAP_PROP_FPS, C.CAMERA_FPS)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        if not self.cap.isOpened():
            raise RuntimeError("camera open failed")

        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = float(self.cap.get(cv2.CAP_PROP_FPS))

        print(
            f"[camera] request={C.CAMERA_WIDTH}x{C.CAMERA_HEIGHT}@{C.CAMERA_FPS}, "
            f"actual={actual_w}x{actual_h}@{actual_fps:.1f}"
        )

        if actual_w != C.CAMERA_WIDTH or actual_h != C.CAMERA_HEIGHT:
            raise RuntimeError(
                f"camera mode changed: expected {C.CAMERA_WIDTH}x{C.CAMERA_HEIGHT}, "
                f"got {actual_w}x{actual_h}"
            )

        for _ in range(C.CAMERA_WARMUP_FRAMES):
            self.cap.read()

        self.running = True
        self.thread = threading.Thread(
            target=self._reader,
            daemon=True,
        )
        self.thread.start()

        # 等第一帧
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            frame, _, _ = self.read(copy=False)
            if frame is not None:
                return self
            time.sleep(0.01)

        raise RuntimeError("camera started but no frame received")

    def _reader(self):
        """后台连续读图；成功时一起更新画面、帧号和单调时钟时间。"""
        while self.running:
            ok, frame = self.cap.read()

            if not ok or frame is None:
                time.sleep(0.005)
                continue

            ts = time.monotonic()

            with self.lock:
                self.frame = frame
                self.frame_id += 1
                self.timestamp = ts

    def read(self, copy=True):
        """在同一把锁内取得画面、帧号和时间，默认复制画面供调用者使用。"""
        with self.lock:
            if self.frame is None:
                return None, self.frame_id, self.timestamp

            frame = self.frame.copy() if copy else self.frame
            return frame, self.frame_id, self.timestamp

    def stop(self):
        """通知采集线程结束，最多等待一秒，再释放摄像头资源。"""
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=1.0)

        if self.cap is not None:
            self.cap.release()

        self.thread = None
        self.cap = None

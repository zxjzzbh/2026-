#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""识别起点蓝色挡板，确认它出现后再等待移开，产生一次发车信号。"""

from collections import deque
from dataclasses import dataclass

import cv2
import numpy as np

import config as C


@dataclass
class BarrierResult:
    """一帧的挡板判断：状态、蓝色面积比例、是否存在以及是否刚可发车。"""

    state: str
    blue_ratio: float
    barrier_present: bool
    go: bool
    mask: np.ndarray


class BlueBarrierGate:
    """
    两阶段起点门：

      SEEK_BARRIER
          ↓ 5帧中至少3帧检测到蓝板
      ARMED
          ↓ 连续3帧蓝板消失
      GO

    这样不会因为程序启动时刚好没检测到蓝色，就直接误发车。
    """

    SEEK_BARRIER = "SEEK_BARRIER"
    ARMED = "ARMED"
    GO = "GO"

    def __init__(self):
        """从寻找挡板开始，准备多帧确认记录和连续消失计数。"""
        self.state = self.SEEK_BARRIER
        self.arm_hist = deque(maxlen=C.BARRIER_ARM_WINDOW)
        self.clear_streak = 0

    @staticmethod
    def _mask(frame):
        """按配置筛出蓝色像素并去除小噪点，返回与输入等大的黑白掩膜。"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower = np.array(C.BARRIER_BLUE_LOWER, dtype=np.uint8)
        upper = np.array(C.BARRIER_BLUE_UPPER, dtype=np.uint8)

        mask = cv2.inRange(hsv, lower, upper)

        # 小量形态学去噪，不改变主体面积太多
        kernel = np.ones((3, 3), np.uint8)
        mask = cv2.morphologyEx(
            mask,
            cv2.MORPH_OPEN,
            kernel,
            iterations=1,
        )

        return mask

    def update(self, frame):
        """用当前画面推进挡板状态，只在首次满足移开条件时返回发车信号。"""
        mask = self._mask(frame)

        blue_ratio = cv2.countNonZero(mask) / float(mask.shape[0] * mask.shape[1])

        present = blue_ratio >= C.BARRIER_BLUE_RATIO_THRESHOLD

        go = False

        if self.state == self.SEEK_BARRIER:
            self.arm_hist.append(1 if present else 0)

            if (
                len(self.arm_hist) == C.BARRIER_ARM_WINDOW
                and sum(self.arm_hist) >= C.BARRIER_ARM_HITS
            ):
                self.state = self.ARMED
                self.clear_streak = 0

        elif self.state == self.ARMED:
            if present:
                self.clear_streak = 0
            else:
                self.clear_streak += 1

                if self.clear_streak >= C.BARRIER_CLEAR_FRAMES:
                    self.state = self.GO
                    go = True

        return BarrierResult(
            state=self.state,
            blue_ratio=float(blue_ratio),
            barrier_present=bool(present),
            go=bool(go),
            mask=mask,
        )

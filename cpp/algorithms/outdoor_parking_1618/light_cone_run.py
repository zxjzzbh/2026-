#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""两项串联入口：先红灯停车、绿灯放行，再巡线和依次绕红桶。

复用fusion_run的相机、电机、缓升和日志，全程只开一次设备。
默认观察；--drive才驱动车轮。手动或蓝板起步均可，不执行斑马线和入库。
"""
from dataclasses import asdict
import json
from pathlib import Path

import cv2
import config as C
from competition_config import defaults, validate, validate_camera_pose
from fusion_logic import FusionController, MissionPolicy
from lane_follow import LaneFollower
from traffic_light import TrafficLightDetector, LightResult


def load_light_config(path):
    """读取已经调好的灯色参数和云台位置；本任务不需要灯架距离标定。"""
    cal = json.loads(Path(path).read_text(encoding='utf-8'))
    cfg = defaults()
    cfg['traffic'] = cal['traffic']
    validate(cfg)
    actual_pose = json.loads(C.CAMERA_POSE_FILE.read_text(encoding='utf-8'))
    validate_camera_pose(actual_pose, cal['camera_pose'])
    return cal


class LightConePolicy(MissionPolicy):
    """灯前固定方向；停车等绿灯，之后把控制交给原巡线绕桶判断。"""

    STOPPED = MissionPolicy.STOPPED + ('LIGHT_BRAKE', 'LIGHT_WAIT')

    def __init__(self, settings, seek_seconds=8.):
        """从直行找灯开始；绕桶阶段沿用原参数和状态，不另写一套动作。"""
        super().__init__(settings)
        self.stage, self.state = 'TRAFFIC', 'LIGHT_SEEK'
        self.light = LightResult()
        self.seek_seconds = seek_seconds
        self.light_start = self.light_last_seen = None
        self.light_fault = ''
        self.completed_cones = set()
        self.last_cone_id = None

    def _light_brake(self, now, reason, fault=False):
        """立即制动；异常制动结束后锁定停止，红灯制动结束后等待绿灯。"""
        self.state, self.owner, self.reason = 'LIGHT_BRAKE', 'none', reason
        self.brake_end = now + C.BRAKE_TIME
        self.light_fault = reason if fault else ''
        return self._command(C.BRAKE_DUTY, reason)

    def _release(self, now):
        """一次性切换到绕桶任务；下一帧先检查当前白线和桶，再允许前进。"""
        self.stage, self.state, self.owner = 'CONES', 'REACQUIRE', 'none'
        self.reacquire_since = now
        self.cone_since = None
        self.reason = ''
        return self._command(C.STOP_DUTY, 'GREEN_CONFIRMED')

    def update(self, now, lane, cone, cross):
        """先完成红绿灯任务，只有绿灯放行后才能进入原巡线绕桶流程。"""
        if self.state in ('HALT', 'DONE'):
            return super().update(now, lane, cone, None)
        if self.stage == 'CONES':
            target = getattr(cone, 'target_id', None)
            if target is not None:
                self.last_cone_id = target
            if cone.reason == 'RESUME_STRAIGHT' and self.last_cone_id is not None:
                self.completed_cones.add(self.last_cone_id)
                self.last_cone_id = None
            return super().update(now, lane, cone, None)

        # 等灯时也累计当前可靠白线；不会因灯前没有白线而禁止首次直行。
        self.lane_hits = self.lane_hits + 1 if lane.valid else 0
        self.last_now = now
        self.steer = float(C.STEER_CENTER_US)
        if self.light_start is None:
            self.light_start = now
        seen = self.light.visible and self.light.color in ('RED', 'YELLOW', 'GREEN')
        green = seen and self.light.color == 'GREEN' and self.light.confirmed
        if self.state == 'LIGHT_BRAKE':
            if now < self.brake_end:
                return self._command(C.BRAKE_DUTY, self.reason)
            if self.light_fault:
                return self.halt(self.light_fault)
            self.state, self.owner = 'LIGHT_WAIT', 'none'
            self.lane_hits = 0
            return self._command(C.STOP_DUTY, 'WAIT_CONFIRMED_GREEN')
        if self.state == 'LIGHT_WAIT':
            if green:
                return self._release(now)
            return self._command(C.STOP_DUTY, 'WAIT_CONFIRMED_GREEN')
        if seen and self.light.color == 'RED':
            return self._light_brake(now, 'RED_SEEN')
        if green:
            return self._release(now)
        if seen:
            self.light_last_seen = now
            self.state = 'LIGHT_APPROACH'
        elif self.light_last_seen is not None:
            if now - self.light_last_seen >= .4:
                return self._light_brake(now, 'LIGHT_LOST', fault=True)
        elif now - self.light_start >= self.seek_seconds:
            return self._light_brake(now, 'LIGHT_NOT_ACTIVATED', fault=True)
        self.owner = 'fixed'
        return self._command(self.s.cruise_duty, 'SEEK_LIGHT' if not seen else 'WAIT_LIGHT_COLOR')


class LightConeController(FusionController):
    """把本帧灯色交给串联判断，绿灯后才让锥桶参与转向。"""

    def __init__(self, settings, cal, cone_params=None, seek_seconds=8., **kwargs):
        """复用巡线绕桶检测器；额外增加已经调好的灯色检测器。"""
        if settings.crosswalk_enabled:
            raise ValueError('两项串联必须关闭斑马线任务')
        super().__init__(settings, cone_params=cone_params, **kwargs)
        self.policy = LightConePolicy(settings, seek_seconds)
        self.light_cfg = cal['traffic']
        self.traffic = TrafficLightDetector(self.light_cfg)
        self.light_result = LightResult()
        self.light_updated = False

    def cones_enabled(self):
        """等灯期间红色灯泡不能被当作要绕的红桶。"""
        return self.policy.stage == 'CONES'

    def cross_enabled(self):
        """本入口只串联红绿灯和绕桶，不执行斑马线停车。"""
        return False

    def update_extra(self, frame, frame_id, captured_at, now):
        """灯前逐帧识别；交接后不再让身后的红绿灯重复打断绕桶。"""
        self.light_updated = self.policy.stage == 'TRAFFIC'
        if self.policy.stage == 'TRAFFIC':
            self.light_result = self.traffic.update(frame)
            self.policy.light = self.light_result

    def step(self, *args):
        """刚停住时清掉旧绿灯确认和行驶中的旧车道，使用停车后的新图重新确认。"""
        previous = self.policy.state
        cmd = super().step(*args)
        if cmd is not None and cmd.state == 'LIGHT_WAIT' and previous != 'LIGHT_WAIT':
            self.traffic.reset_confirmation()
            self.light_result.confirmed = False
            if isinstance(self.lane, LaneFollower):
                self.lane = LaneFollower()
        return cmd

    def extra_event(self):
        """记录任务阶段、灯色和已完成的桶数，方便区分谁在控制车。"""
        return dict(mission_stage=self.policy.stage, traffic=asdict(self.light_result),
                    traffic_updated=self.light_updated,
                    completed_cone_maneuvers=len(self.policy.completed_cones))

    def draw_extra(self, frame):
        """灯前画灯框，绕桶时只标任务阶段，避免显示过期灯框。"""
        out = (self.traffic.draw(frame, self.light_result, self.light_cfg)
               if self.policy.stage == 'TRAFFIC' else frame.copy())
        cv2.putText(out, self.policy.stage, (5, 106), cv2.FONT_HERSHEY_SIMPLEX,
                    .4, (0, 255, 255), 1)
        return out


def main(argv=None):
    """启动同一个相机和驱动循环；参数、反馈包和退出停车都交给共用入口。"""
    from fusion_run import main as run_fusion
    return run_fusion(argv, field=True, light_cone=True)


if __name__ == '__main__':
    raise SystemExit(main())

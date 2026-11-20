"""三个运行入口共用的主循环：准备设备、取图、计算命令、驱动车轮、保存日志。

默认只观察；--drive才启动车轮，--video用于离线回放。
比赛入口是competition_run.py；原操场联调入口是fusion_field_run.py。"""

import argparse
from contextlib import contextmanager
from dataclasses import asdict
from datetime import datetime
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time
import zipfile
import cv2
from fusion_snapshots import transition_events, save_snapshot
import config as C
from barrier import BlueBarrierGate
from cone_solo import CarIO, checked, restore_gimbal
from cone_solo_logic import Params, draw_debug as draw_cones
from crosswalk import CrosswalkDetector
from lane_follow import LaneFollower, LANE_VERSION
from wisecar_vision import SOURCE_COMMIT
from fusion_logic import FusionController, Settings


@contextmanager
def runtime_config(field, lane_roi=None):
    """为操场和比赛入口临时启用实测参数，结束后恢复原值。

    这里的值优先于config.py同名参数，只在本次运行期间生效；
    因此同一个Python进程里的观察、回放不会互相残留配置。
    lane_roi可为操场测试指定画面上下范围，结束或报错时同样恢复。"""
    values = (
        {
            'LANE_ROI_Y0_RATIO': 0.35,
            'LANE_ROI_Y1_RATIO': 0.65,
              # Measure farther below the horizon, on observed track paint.
              # With the master ROI this is y=189 rather than y=173 at 288p.
              'LANE_PAIR_REF_RATIO': 0.80,
            'CROSS_ROI_Y0': 0.4,
            'CROSS_S_MAX': 65,
            'CROSS_REJECT_TOP_CLIPPED': True,
            'CROSS_IDLE_TRACK_EXPIRE_FRAMES': 5,
            'CROSS_CONFIRMED_CLOSE_BRAKE': False,
            'CROSS_SHADOW_ROW_RECOVERY': True,
            'CROSS_SHADOW_MIN_TOP': 0.45,
            'CROSS_USE_STRIPE_TOP': True,
            'CROSS_EARLY_STOP_ON_ARM': False,
            'CROSS_CLOSE_SHAPE_TOP': 0.68,
            'CROSS_APPROACH_STRIPE_TOP': 0.48,
              'CROSS_BRAKE_STRIPE_TOP': 0.52,
        }
        if field
        else {}
    )
    if lane_roi is not None:
        values['LANE_ROI_Y0_RATIO'], values['LANE_ROI_Y1_RATIO'] = lane_roi
    missing = object()
    previous = {k: getattr(C, k, missing) for k in values}
    try:
        for k, v in values.items():
            setattr(C, k, v)
        yield
    finally:
        for k, v in previous.items():
            if v is missing:
                delattr(C, k)
            else:
                setattr(C, k, v)


class FusionIO(CarIO):
    """把总判断给出的命令真正发给车轮和转向。

    前进动力逐步增加，减速和刹车立即下发；出错时沿用CarIO的锁定停止。"""

    def __init__(self, c, *args, **kwargs):
        if not (math.isfinite(c.CROSS_FR_BRAKE_DUTY)
                and 10200 <= c.CROSS_FR_BRAKE_DUTY < c.STOP_DUTY
                and math.isfinite(c.CROSS_FR_BRAKE_TIME)
                and 0 < c.CROSS_FR_BRAKE_TIME <= 0.5):
            raise ValueError('斑马线F/R参数无效：反向10200至中位以下，时长0至0.5秒')
        self._cross_brake_until = None
        super().__init__(c, *args, **kwargs)

    def check_watchdog(self):
        # The existing 25 ms watcher also ends a crosswalk pulse when camera
        # processing stalls. This is a software deadline, not a hardware timer.
        with self.lock:
            if (self.enabled and self._cross_brake_until is not None
                    and self.clock() >= self._cross_brake_until
                    and self.current < self.c.STOP_DUTY):
                try:
                    checked(self.pi.set_PWM_dutycycle(self.c.MOTOR_GPIO, self.c.STOP_DUTY))
                    self.current = self.c.STOP_DUTY
                except Exception:
                    self.stop('GPIO_ERROR')
            return super().check_watchdog()

    def write_command(self, cmd, captured_at):
        """检查命令和图片时间后，下发转向及动力；刹车不经过前进缓升，出错则锁定停车。"""
        with self.lock:
            if cmd.state in ('HALT', 'DONE'):
                self.stop(cmd.reason or cmd.state)
                return
            if self.reason or not self.enabled:
                raise RuntimeError('未使能或停车已锁定')
            now = self.clock()
            if (
                not math.isfinite(captured_at)
                or not 0 <= now - captured_at <= C.MAX_FRAME_AGE_SECONDS
            ):
                self.stop('STALE_COMMAND')
                raise RuntimeError('处理完成后图像已过期')
            braking = cmd.state in ('BRAKE', 'FAULT_BRAKE', 'LIGHT_BRAKE', 'PARK_BRAKE')
            saved_brake = ((cmd.state == 'BRAKE' and cmd.owner == 'crosswalk_saved')
                           or (cmd.state == 'LIGHT_BRAKE' and cmd.owner == 'traffic_saved'))
            brake_duty = C.BRAKE_DUTY
            if cmd.state in ('BRAKE', 'LIGHT_BRAKE'):
                parking_or_manual = cmd.owner == 'parking' or cmd.reason in ('MANUAL_PAUSE', 'MANUAL_SKIP', 'PARK_ENTRY_SETTLE_BRAKE')
                brake_duty = C.PARK_MANUAL_BRAKE_DUTY if parking_or_manual else C.CROSS_FR_BRAKE_DUTY
            if saved_brake:
                brake_duty = 10600
            allowed = (
                cmd.duty == brake_duty
                if braking
                else C.STOP_DUTY <= cmd.duty <= 16400
            )
            if not allowed or not math.isfinite(cmd.steer_us):
                self.stop('INVALID_COMMAND')
                raise ValueError('非法驱动命令')
            if braking or cmd.duty <= self.current or cmd.duty == C.STOP_DUTY:
                target = cmd.duty
            else:
                dt = max(0.0, min(0.1, now - self.last_command))
                target = min(cmd.duty, max(C.STOP_DUTY, self.current) + 600.0 * dt)
            first_cross_brake = cmd.state in ('BRAKE', 'LIGHT_BRAKE') and self._cross_brake_until is None
            if cmd.state not in ('BRAKE', 'LIGHT_BRAKE'):
                self._cross_brake_until = None
            elif first_cross_brake and self.current <= C.STOP_DUTY:
                # No forward output to arrest: do not launch backwards from rest.
                self._cross_brake_until = now
                target = C.STOP_DUTY
            elif self._cross_brake_until is not None and now >= self._cross_brake_until:
                target = C.STOP_DUTY
            pulse = int(max(C.STEER_MIN_US, min(C.STEER_MAX_US, cmd.steer_us)))
            try:
                checked(self.pi.set_servo_pulsewidth(C.STEER_GPIO, pulse))
                checked(self.pi.set_PWM_dutycycle(C.MOTOR_GPIO, int(target)))
                if first_cross_brake and target < C.STOP_DUTY:
                    self._cross_brake_until = self.clock() + (.25 if saved_brake else C.CROSS_FR_BRAKE_TIME)
            except BaseException:
                self.stop('GPIO_ERROR')
                raise
            self.current = int(target)
            self.peak = max(self.peak, self.current)
            self.last_command = now


def sync_file(path):
    """把已写入的文件刷新到磁盘，减少异常断电后只剩空文件的情况。"""
    with path.open('r+b') as saved:
        os.fsync(saved.fileno())


def parse_args(argv=None, field=False, competition=False, light_cone=False):
    """读取命令行选项并检查矛盾设置，例如离线视频不能同时启动车轮；返回本次选项。"""
    description = (
        '操场联调：默认观察、动力12600、按白线约束绕桶并确认回正、限时60秒；--drive启用电机。'
        if field
        else __doc__
    )
    if competition:
        description = '比赛入口：蓝板移开→斑马线停车10秒→红绿灯停车见绿放行→依次绕桶→入空车位。默认观察。'
    if light_cone:
        if not field or competition:
            raise ValueError('两项串联使用独立操场入口')
        description = '两项串联：先直行触发灯，红停绿行，再巡线依次绕桶。默认观察。'
    p = argparse.ArgumentParser(description=description)
    p.add_argument('--drive', action='store_true', help='控制电机；默认只观察')
    p.add_argument('--video', type=Path, help='离线视频，禁止与 --drive 同用')
    p.add_argument(
        '--start',
        choices=('barrier', 'manual'),
        default='barrier' if competition else 'manual' if field else C.START_MODE,
    )
    p.add_argument('--show', action='store_true', help='显示调试窗口，需要桌面/X11')
    p.add_argument(
        '--seconds',
        type=float,
        default=600.0 if competition else 60.0 if field else 0,
        help='运行总秒数；操场入口必须大于0，普通入口0=持续运行',
    )
    p.add_argument('--name', help='本次日志名称；省略则使用时间戳，不覆盖已有记录')
    p.add_argument('--cruise-duty', type=int, default=12600 if field else C.CRUISE_DUTY)
    p.add_argument(
        '--cone-duty', type=int, default=12600 if field else C.LOW_FORWARD_DUTY
    )
    p.add_argument(
        '--approach-duty',
        type=int,
        default=12600 if field else C.CROSSWALK_APPROACH_DUTY,
    )
    p.add_argument('--hold-seconds', type=float, default=10.0)
    p.add_argument(
        '--stop-at-crosswalk', action='store_true',
        help=argparse.SUPPRESS if light_cone else '可选：斑马线保持后结束'
    )
    p.add_argument('--color', choices=('blue', 'red'), default='blue')
    plain_field = field and not competition and not light_cone
    if field and not competition:
        if not light_cone:
            protection = p.add_mutually_exclusive_group()
            protection.add_argument('--cone-protection', dest='cone_protection',
                                    action='store_true', help='启用丢桶、无进展和绕桶超时停车')
            protection.add_argument('--no-cone-protection', dest='cone_protection',
                                    action='store_false', help='取消锥桶保护停车，丢桶后按白线回正')
            p.set_defaults(cone_protection=False)
            p.add_argument(
                '--disable-cones', action='store_true',
                help='只测试巡线和斑马线：关闭锥桶检测、避障及巡线颜色遮罩',
            )
        p.add_argument(
            '--lane-cone-only',
            action='store_true',
            help=argparse.SUPPRESS if light_cone else '只测试巡线和绕桶，关闭斑马线检测、停车和播报',
        )
        p.add_argument(
            '--lane-roi',
            type=float,
            nargs=2,
            metavar=('TOP', 'BOTTOM'),
            help='巡线观察的上下范围；只巡线绕桶默认0.54 0.78，其他操场任务沿用原范围',
        )
        p.add_argument(
            '--cone-roi-top',
            type=float,
            default=0.30,
            help=('从画面55%%高度向下找桶，沿用最近场地测试范围' if light_cone
                  else '从画面多高处向下找桶；默认0.30，即原来的下方70%%'),
        )
    p.add_argument(
        '--front-x',
        type=float,
        default=LaneFollower._load_target_ratio(),
        help='绕桶的车头参考位置；默认与巡线标定一致，可显式覆盖',
    )
    p.add_argument('--turn-us', type=float, default=120.0 if plain_field else 90.0)
    p.add_argument(
        '--recovery-turn-us',
        type=float,
        default=130.0,
        help='让开桶后修正朝向的转向上限，与初始避让幅度分开',
    )
    p.add_argument(
        '--cone-steer-rate',
        type=float,
        default=1800.0 if field else 600.0,
        help='绕桶转向变化速率，单位微秒/秒；同时作用于绕桶和融合输出',
    )
    p.add_argument(
        '--min-turn-us',
        type=float,
        default=85.0 if plain_field else 55.0 if field else 25.0,
        help='避让期间最小转向偏移；实际输出仍受斜率限制',
    )
    p.add_argument(
        '--cone-padding',
        type=float,
        default=0.21 if field else 0.17,
        help='锥桶两侧扩展的图像宽度比例，不是厘米',
    )
    p.add_argument('--align-seconds', type=float, default=0.3)
    p.add_argument(
        '--pass-seconds',
        type=float,
        default=0.2 if field else 0.6,
        help='确认当前桶离开后保持的时间；操场模式此时已开始按车道修正',
    )
    p.add_argument(
        '--cone-lane-margin',
        type=float,
        default=0.22,
        help='绕桶时两侧白线的预留比例，相对于当前车道宽度',
    )
    p.add_argument(
        '--cone-align-timeout',
        type=float,
        default=2.0,
        help='回正超过此秒数后标记继续恢复，不因回正超时停车',
    )
    p.add_argument('--trigger-height', type=float, default=6.0)
    if light_cone:
        p.set_defaults(light_cone=True, lane_cone_only=True, color='red',
                       lane_roi=[0.54, 0.78], cone_roi_top=0.55)
        p.add_argument('--light-config', type=Path,
                       default=Path(__file__).with_name('light_stop_calibration.json'),
                       help='沿用已经调好的灯色参数和云台位置')
        p.add_argument('--light-seek-seconds', type=float, default=8.,
                       help='灯未亮时最多直行的秒数，超时刹车')
    if competition:
        p.add_argument(
            '--competition-config',
            type=Path,
            default=Path(__file__).resolve().parent / 'competition_config.json',
        )
    a = p.parse_args(argv)
    if light_cone and (not math.isfinite(a.light_seek_seconds) or
                       not 0 < a.light_seek_seconds <= 20):
        p.error('--light-seek-seconds需大于0且不超过20')
    if field and not competition:
        if getattr(a, 'disable_cones', False) and a.lane_cone_only:
            p.error('--disable-cones 不能与 --lane-cone-only 同用')
        if a.lane_cone_only and a.lane_roi is None:
            # 使用已回放的学长巡线区域；显式 --lane-roi 仍优先。
            a.lane_roi = [0.35, 0.65]
        if a.lane_roi is not None and not (
            all(math.isfinite(v) for v in a.lane_roi)
            and 0 <= a.lane_roi[0] < a.lane_roi[1] <= 1
        ):
            p.error('--lane-roi需要满足0 <= TOP < BOTTOM <= 1')
        if not math.isfinite(a.cone_roi_top) or not 0 <= a.cone_roi_top < 1:
            p.error('--cone-roi-top需要在0（含）到1（不含）之间')
        if a.lane_cone_only and a.stop_at_crosswalk:
            p.error('只巡线绕桶时不能指定斑马线停车后结束')
    if a.video and a.drive:
        p.error('--video 不能控制电机')
    if not math.isfinite(a.seconds) or a.seconds < 0:
        p.error('--seconds 必须为非负有限数')
    if field and a.seconds <= 0:
        p.error('操场联调必须设置大于0的运行限时')
    if competition and (a.hold_seconds != 10 or a.stop_at_crosswalk):
        p.error('比赛入口使用10秒停车保持并继续后续任务')
    if competition and a.drive and (a.start != 'barrier'):
        p.error('比赛实车入口必须以蓝色挡板移开起步；手动仅用于观察')
    if a.name is not None and (
        not a.name or a.name in ('.', '..') or any((ch in a.name for ch in '/\\:'))
    ):
        p.error('--name 只能是文件夹名')
    return a


def play_audio(path=None):
    """播放给定音频，未指定时用你好你好；播放在后台进行，不挡住相机和停车计时。"""
    path = C.AUDIO_FILE if path is None else Path(path)
    if not path.exists():
        print('[audio] 文件不存在：', path, flush=True)
        return
    try:
        subprocess.Popen(
            ['aplay', '-q', str(path)],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
    except OSError as exc:
        print('[audio]', exc, flush=True)


def draw_frame(frame, ctl, cmd):
    """合并巡线、斑马线、绕桶和比赛阶段的标记，返回供人查看的调试图片。"""
    out = LaneFollower.draw_debug(frame, ctl.lane_result)
    if ctl.cross_result is not None:
        out = CrosswalkDetector.draw_debug(out, ctl.cross_result)
    if ctl.cones_enabled():
        out = draw_cones(out, ctl.cone_result, ctl.cone.p)
    cv2.putText(
        out,
        '%s owner=%s pwm=%d' % (cmd.state, cmd.owner, cmd.duty),
        (4, frame.shape[0] - 8),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.38,
        (0, 255, 255),
        1,
    )
    return ctl.draw_extra(out)


def main(argv=None, field=False, competition=False, light_cone=False):
    """选择本次参数组，再进入运行循环；正常返回0，锁定故障停机返回1。"""
    args = parse_args(argv, field=field, competition=competition, light_cone=light_cone)
    with runtime_config(field, lane_roi=getattr(args, 'lane_roi', None)):
        return _run(argv, field=field, competition=competition, args=args)


def _load_competition_config(args, competition):
    """比赛入口读取并检查JSON；实车模式还核对相机位置。普通入口返回None，检查失败时不启动硬件。"""
    competition_cfg = None
    if competition:
        from competition_config import load, validate, validate_camera_pose

        competition_cfg = load(args.competition_config)
        validate(competition_cfg, drive=args.drive)
        if args.drive:
            pose = json.loads(C.CAMERA_POSE_FILE.read_text(encoding='utf-8'))
            calibrated = competition_cfg.get('camera_pose') or {}
            validate_camera_pose(pose, calibrated)
    return competition_cfg


def _make_run_record(args, settings, params, competition_cfg, field, competition):
    """把本次命令参数、有效配置和云台/巡线标定整理成记录，供事后核对。"""
    metadata = dict(
        revision='WISECAR_LANE_FR_V2',
        lane_detector=LANE_VERSION,
        lane_source_commit=SOURCE_COMMIT,
        profile=(
            'COMPETITION_V1'
            if competition
            else (
                'FIELD_LANE_CONE'
                if getattr(args, 'lane_cone_only', False)
                else 'FIELD_V4' if field else 'FUSION'
            )
        ),
        args={k: str(v) if isinstance(v, Path) else v for k, v in vars(args).items()},
        settings=asdict(settings),
        cone=asdict(params),
        config={
            k: v
            for k, v in vars(C).items()
            if k.isupper() and isinstance(v, (str, int, float, bool, tuple, dict))
        },
    )
    if competition_cfg is not None:
        metadata['competition'] = competition_cfg
    if getattr(args, 'light_cone', False):
        metadata['revision'] = metadata['profile'] = 'LIGHT_CONE_V1'
    for key, path in (
        ('camera_pose', C.CAMERA_POSE_FILE),
        ('lane_target', C.LANE_TARGET_FILE),
    ):
        metadata[key] = (
            json.loads(path.read_text(encoding='utf-8')) if path.exists() else None
        )
    return metadata


def _print_run_settings(metadata, settings, params, args, folder, competition):
    """在终端列出本次动力、绕桶参数、运行模式和任务顺序。"""
    print('[revision]', metadata['revision'], flush=True)
    print(
        '[profile]',
        metadata['profile'],
        'cruise=',
        settings.cruise_duty,
        'cone=',
        settings.cone_duty,
        'approach=',
        settings.approach_duty,
        flush=True,
    )
    print(
        '[cone config] min=%.0f max=%.0f padding=%.3f hold_peak=%s'
        % (
            params.min_turn_us,
            params.max_turn_us,
            params.padding,
            params.hold_peak_turn,
        ),
        flush=True,
    )
    print('[lane detector] ' + LANE_VERSION, flush=True)
    if not settings.cones_enabled:
        print('[cones] DISABLED (本次不检测锥桶、不绕桶、不遮罩巡线画面)', flush=True)
    elif not settings.cone_protection_enabled:
        print('[cone protection] DISABLED (丢桶、无进展、绕桶超时不停车；丢桶按白线回正)', flush=True)
    print('[cone steering] slew=%.0f us/s' % params.slew_us_s, flush=True)
    print(
        '[cone recovery] lane=%s front=%.4f margin=%.2f pass=%.2fs timeout=%.2fs turn=%.0f'
        % (
            params.lane_guidance,
            params.front_x,
            params.lane_margin_ratio,
            params.pass_seconds,
            params.align_timeout,
            params.recovery_turn_us,
        ),
        flush=True,
    )
    print(
        '[vision] lane=%.3f..%.3f cone_top=%.3f (画面高度比例)'
        % (C.LANE_ROI_Y0_RATIO, C.LANE_ROI_Y1_RATIO, params.roi_top),
        flush=True,
    )
    print(
        '[mode]',
        'DRIVE' if args.drive else 'OBSERVE (不写电机/转向)',
        '[log]',
        folder,
        flush=True,
    )
    if getattr(args, 'light_cone', False):
        print('[flow] 手动/挡板起步 -> 直行触发亮灯 -> 红灯立即刹车 -> 等绿灯确认 -> 巡线/依次绕桶', flush=True)
        print('[light] 初始找灯限时%.1fs；绿灯后继续绕桶，不使用单项测试的3秒结束' % args.light_seek_seconds, flush=True)
    elif competition:
        print(
            '[flow] 蓝板移开 -> 斑马线停车播报10秒 -> 红绿灯停车见绿放行 -> 单桶依次绕行 -> 无挡板车位停车',
            flush=True,
        )
    elif not settings.crosswalk_enabled:
        print(
            '[flow] 手动/挡板起步 -> 巡线 -> 单桶绕行 -> 恢复巡线/下一个桶', flush=True
        )
        print('[cross] DISABLED (本次仅巡线绕桶，不检测斑马线、不播报)', flush=True)
    else:
        print(
            '[flow] 起步 -> %s -> 斑马线停车 %.1fs -> %s'
            % ('巡线/绕桶' if settings.cones_enabled else '巡线',
               settings.hold_seconds,
               ('恢复巡线/绕桶' if settings.cones_enabled else '恢复巡线') if settings.resume else '结束'),
            flush=True,
        )
    if settings.crosswalk_enabled and getattr(C, 'CROSS_USE_STRIPE_TOP', False):
        print(
            '[cross config] approach=%.3f brake=%.3f (图像位置比例)'
            % (C.CROSS_APPROACH_STRIPE_TOP, C.CROSS_BRAKE_STRIPE_TOP),
            flush=True,
        )
    if settings.crosswalk_enabled:
        print('[cross brake] F/R pulse=%d duration=%.2fs -> neutral=%d; ESC switch must be F/R'
              % (C.CROSS_FR_BRAKE_DUTY, C.CROSS_FR_BRAKE_TIME, C.STOP_DUTY), flush=True)


def _make_frame_record(ctl, cmd, fid, now, ts, begin, capture, car, before_cross):
    """把本帧检测结果、计划动力和实际下发动力放到同一条日志中；只整理数据，不控制车。"""
    lr, cr, co = (ctl.lane_result, ctl.cross_result, ctl.cone_result)
    event = dict(
        frame=fid,
        t=now - begin,
        age_ms=(now - ts) * 1000,
        processed_age_ms=(time.monotonic() - ts) * 1000 if capture is None else 0.0,
        **asdict(cmd),
        actual_duty=car.current if car else None,
        lane_detector=LANE_VERSION,
        lane_mode=lr.mode if lr else None,
        lane_valid=getattr(lr, 'valid', False),
        lane_lost_count=getattr(lr, 'lost_count', None),
        error_px=getattr(lr, 'error_px', None),
        lane_center_x=getattr(lr, 'center_x', None),
        lane_target_x=getattr(lr, 'target_x', None),
        lane_width=getattr(lr, 'lane_width', None),
        lane_measured_center_x=getattr(lr, 'measured_center_x', None),
        lane_left_x=getattr(lr, 'left_x', None),
        lane_right_x=getattr(lr, 'right_x', None),
        lane_heading_valid=getattr(lr, 'heading_valid', False),
        lane_heading_error_px=getattr(lr, 'heading_error_px', None),
        lane_left_line=getattr(lr, 'left_line', None),
        lane_right_line=getattr(lr, 'right_line', None),
        cone_state=co.state if co else None,
        cone_target_id=getattr(co, 'target_id', None),
        cone_turn=getattr(co, 'turn_us', None),
        cone_reason=getattr(co, 'reason', None),
        cone_edge_limited=getattr(ctl.cone, 'edge_limited', False),
        cone_align_hits=getattr(ctl.cone, 'align_hits', 0),
        cone_pass_confirmed=getattr(ctl.cone, 'pass_confirmed', False),
        cone_exit_evidence=sorted(getattr(ctl.cone, 'exit_evidence', set())),
        cone_handoff_turn_us=getattr(ctl.cone_result, 'handoff_turn_us', None),
        boxes=getattr(co, 'boxes', []),
        cross_state=cr.state if cr else None,
        cross_position=getattr(cr, 'distance_tracked', None),
        cross_position_raw=getattr(cr, 'distance_raw', None),
        cross_metric=getattr(cr, 'distance_metric', None),
        cross_seen=getattr(cr, 'far_detected', False),
        cross_valid_count=getattr(cr, 'valid_count', 0),
        cross_updated=cr is not None and cr is not before_cross,
        near_bottom=getattr(cr, 'near_bottom_tracked', None),
    )
    event.update(ctl.extra_event())
    return event


def _print_frame_progress(fid, cmd, event, cr, competition):
    """打印本帧状态、实际动力、灯色和斑马线位置，便于人在终端观察。"""
    print(
        'f=%06d state=%s owner=%s steer=%d duty=%d actual=%s lane=%s reason=%s'
        % (
            fid,
            cmd.state,
            cmd.owner,
            cmd.steer_us,
            cmd.duty,
            event['actual_duty'],
            event['lane_mode'],
            cmd.reason,
        ),
        flush=True,
    )
    if 'mission_stage' in event and not competition:
        print('[mission]', event['mission_stage'], 'light=', event['traffic']['color'],
              'confirmed=', event['traffic']['confirmed'],
              'light_updated=', event['traffic_updated'],
              'cone_done=', event['completed_cone_maneuvers'], flush=True)
    if competition:
        print(
            '[mission]',
            event['mission_stage'],
            'light=',
            event['traffic']['color'],
            'lamp_width=',
            round(event['traffic']['width_ratio'], 3),
            'cone_done=',
            event['completed_cone_maneuvers'],
            'parking=',
            event['parking']['blocked'],
            flush=True,
        )
    if cr is not None:
        print(
            '[cross] state=%s metric=%s pos=%.3f raw=%.3f seen=%s updated=%s'
            % (
                cr.state,
                cr.distance_metric,
                cr.distance_tracked,
                cr.distance_raw,
                event['cross_seen'],
                event['cross_updated'],
            ),
            flush=True,
        )


def _finish_logs(folder, status, frames):
    """硬件已经关闭后，写退出原因和帧数，把日志与图片打成反馈包。"""
    (folder / 'summary.json').write_text(
        json.dumps(dict(status=status, frames=frames), ensure_ascii=False, indent=2),
        encoding='utf-8',
    )
    sync_file(folder / 'summary.json')
    if (folder / 'events.jsonl').exists():
        sync_file(folder / 'events.jsonl')
    archive = folder.with_name(folder.name + '.zip')
    with zipfile.ZipFile(archive, 'x', zipfile.ZIP_DEFLATED) as z:
        for path in folder.iterdir():
            z.write(path, arcname=folder.name + '/' + path.name)
    sync_file(archive)
    print('[exit]', status, '[feedback ZIP]', archive, flush=True)


def _run(argv=None, field=False, competition=False, args=None):
    """按顺序完成准备、取图、判断、下发命令和保存记录。

    只有--drive才创建电机控制对象。每张图先检查是否过期，
    计算完成后再检查一次；退出时总是先停机，再释放相机和打包日志。"""
    # 先核对参数和标定，再接触硬件，避免配置有问题时已经发车。
    if args is None:
        args = parse_args(argv, field=field, competition=competition)
    competition_cfg = _load_competition_config(args, competition)
    light_cal = None
    if getattr(args, 'light_cone', False):
        from light_cone_run import load_light_config
        light_cal = load_light_config(args.light_config)
    settings = Settings(
        cruise_duty=args.cruise_duty,
        cone_duty=args.cone_duty,
        approach_duty=args.approach_duty,
        hold_seconds=args.hold_seconds,
        cone_steer_slew_us_s=args.cone_steer_rate,
        resume=not args.stop_at_crosswalk,
        crosswalk_enabled=not getattr(args, 'lane_cone_only', False),
        cones_enabled=not getattr(args, 'disable_cones', False),
        brake_on_cone_fault=field,
        cone_protection_enabled=getattr(args, 'cone_protection', True),
    )
    params = Params(
        color=args.color,
        roi_top=getattr(args, 'cone_roi_top', 0.30),
        front_x=args.front_x,
        max_turn_us=args.turn_us,
        min_turn_us=args.min_turn_us,
        padding=args.cone_padding,
        hold_peak_turn=field,
        slew_us_s=args.cone_steer_rate,
        align_seconds=args.align_seconds,
        pass_seconds=args.pass_seconds,
        trigger_height=args.trigger_height,
        lane_guidance=field,
        lane_margin_ratio=args.cone_lane_margin,
        align_timeout=args.cone_align_timeout,
        recovery_turn_us=args.recovery_turn_us,
        protection_stop=settings.cone_protection_enabled,
    )
    root = Path(__file__).resolve().parent / 'fusion_debug'
    name = args.name or datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    folder = root / name
    folder.mkdir(parents=True, exist_ok=False)
    metadata = _make_run_record(
        args, settings, params, competition_cfg, field, competition
    )
    if light_cal is not None:
        metadata['light_calibration'] = light_cal
    (folder / 'run.json').write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2), encoding='utf-8'
    )
    sync_file(folder / 'run.json')
    _print_run_settings(metadata, settings, params, args, folder, competition)
    car = camera = capture = None
    status = 'STARTING'
    if light_cal is not None:
        from light_cone_run import LightConeController
        ctl = LightConeController(settings, light_cal, cone_params=params,
                                  seek_seconds=args.light_seek_seconds)
    elif competition:
        from competition_logic import CompetitionController

        ctl = CompetitionController(settings, competition_cfg, cone_params=params)
    else:
        ctl = FusionController(settings, cone_params=params)
    gate = BlueBarrierGate() if args.start == 'barrier' else None
    gate_clear_at = None
    last_gate_id = None
    last_state = None
    previous_snapshot_event = None
    last_snapshot_frame = last_snapshot_debug = last_snapshot_event = None
    next_save = 0.0
    next_sync = 0.0
    fid = -1
    frames = 0
    begin = time.monotonic()
    previous = {}

    def interrupted(signum, frame):
        """收到程序终止或终端断开信号时，转入统一的停车和收尾流程。"""
        raise KeyboardInterrupt

    try:
        for signame in ('SIGTERM', 'SIGHUP'):
            sig = getattr(signal, signame, None)
            if sig is not None:
                previous[sig] = signal.signal(sig, interrupted)
        if args.video:
            capture = cv2.VideoCapture(str(args.video))
            if not capture.isOpened():
                raise RuntimeError('无法打开视频：' + str(args.video))
            fps = capture.get(cv2.CAP_PROP_FPS)
            if not math.isfinite(fps) or fps <= 0:
                raise ValueError('视频 FPS 无效')
        else:
            from camera_stream import LatestFrameCamera

            if args.drive:
                car = FusionIO(C)
            restore_gimbal(C)
            if args.start == 'manual':
                input(
                    '手动起步：准备完成后按 Enter 开始'
                    + ('实车运行：' if args.drive else '观察：')
                )
            camera = LatestFrameCamera()
            camera.start()
        # 设备准备好以后开始计时；下面每次处理一张新画面。
        begin = time.monotonic()
        with (folder / 'events.jsonl').open('w', encoding='utf-8', buffering=1) as log:
            while True:
                if capture is not None:
                    ok, frame = capture.read()
                    if not ok:
                        status = 'VIDEO_END'
                        break
                    fid += 1
                    now = ts = begin + fid / fps
                else:
                    frame, fid, ts = camera.read()
                    now = time.monotonic()
                if args.seconds and now - begin >= args.seconds:
                    status = 'TIME_LIMIT'
                    break
                if car is not None and car.reason:
                    status = car.reason
                    ctl.policy.halt(status)
                    break
                if (
                    frame is None
                    or not math.isfinite(ts)
                    or (not 0 <= now - ts <= settings.max_age)
                ):
                    status = 'STALE_FRAME'
                    ctl.policy.halt(status)
                    if car:
                        car.stop(status)
                    break
                # 起点挡板还没放行时，只检查挡板，不进入行驶判断。
                if gate is not None:
                    if fid == last_gate_id:
                        time.sleep(0.002)
                        continue
                    last_gate_id = fid
                    gr = gate.update(frame)
                    log.write(
                        json.dumps(
                            dict(
                                frame=fid,
                                t=now - begin,
                                state='WAIT_BARRIER',
                                gate=gr.state,
                                blue_ratio=gr.blue_ratio,
                            )
                        )
                        + '\n'
                    )
                    if now >= next_sync:
                        log.flush()
                        os.fsync(log.fileno())
                        next_sync = now + 0.5
                    if gr.go and gate_clear_at is None:
                        gate_clear_at = now + C.BARRIER_CLEAR_SETTLE_SECONDS
                    if gate_clear_at is None or now < gate_clear_at:
                        continue
                    print('[start] BLUE BARRIER -> GO', flush=True)
                    gate = None
                # 检测和总判断只给出建议，这一步还没有写电机。
                before_cross = ctl.cross_result
                cmd = ctl.step(frame, fid, ts, now)
                if cmd is None:
                    time.sleep(0.002)
                    continue
                if capture is None and time.monotonic() - ts > settings.max_age:
                    cmd = ctl.policy.halt('PROCESSING_TOO_SLOW')
                # 画面计算期间也可能变旧，确认仍及时以后才下发命令。
                if car is not None:
                    if not car.enabled and cmd.duty > C.STOP_DUTY:
                        car.arm()
                    if car.enabled or cmd.state in ('HALT', 'DONE'):
                        car.write_command(cmd, ts)
                # 从命令实际下发后开始算刹车/保持时间，避免把计算耗时算进去。
                applied_at = time.monotonic() if capture is None else now
                if (
                    cmd.state in ('BRAKE', 'FAULT_BRAKE', 'LIGHT_BRAKE', 'PARK_BRAKE')
                    and last_state != cmd.state
                ):
                    duration = C.CROSS_FR_BRAKE_TIME if cmd.state in ('BRAKE', 'LIGHT_BRAKE') else C.BRAKE_TIME
                    ctl.policy.brake_end = applied_at + duration
                if cmd.state == 'HOLD' and last_state != 'HOLD':
                    ctl.policy.hold_end = applied_at + settings.hold_seconds
                if cmd.announce and args.drive:
                    if competition:
                        play_audio(competition_cfg['announcement_file'])
                    else:
                        play_audio()
                frames += 1
                # 命令发出后再记日志，同时记目标动力和真正下发的动力。
                event = _make_frame_record(
                    ctl, cmd, fid, now, ts, begin, capture, car, before_cross
                )
                log.write(json.dumps(event, ensure_ascii=False) + '\n')
                phase_tags = transition_events(previous_snapshot_event, event)
                previous_snapshot_event = event
                changed = cmd.state != last_state
                snapshot_due = bool(phase_tags) or now >= next_save or cmd.state in ('HALT', 'DONE')
                if snapshot_due or args.show:
                    dbg = draw_frame(frame, ctl, cmd)
                    if snapshot_due:
                        save_snapshot(folder, frame, dbg, event, phase_tags, args.drive)
                        last_snapshot_frame = frame.copy()
                        last_snapshot_debug = dbg.copy()
                        last_snapshot_event = dict(event)
                        next_save = now + 1.0
                if changed or now >= next_sync:
                    log.flush()
                    os.fsync(log.fileno())
                    next_sync = now + 0.5
                if changed or frames % C.LOG_EVERY_N_FRAMES == 0:
                    _print_frame_progress(
                        fid, cmd, event, ctl.cross_result, competition
                    )
                status = cmd.reason or cmd.state
                if cmd.state in ('HALT', 'DONE'):
                    if competition and cmd.reason == 'PARKED_PAYMENT_BY_OPERATOR':
                        print(
                            '[payment] 已输出停车指令。请队员在停车后30秒内远程扫码支付0.01元。',
                            flush=True,
                        )
                    break
                if args.show:
                    cv2.imshow('Fusion', dbg)
                    if cv2.waitKey(1) & 255 in (ord('q'), 27):
                        status = 'USER_STOP'
                        break
                last_state = cmd.state
    except KeyboardInterrupt:
        status = 'INTERRUPTED'
    except Exception as exc:
        status = 'ERROR: ' + str(exc)
        raise
    finally:
        # 无论正常结束、手动中断还是报错，都先停机，再整理相机和日志。
        try:
            if car is not None:
                car.close()
        finally:
            try:
                if camera is not None:
                    camera.stop()
                if capture is not None:
                    capture.release()
                if args.show:
                    cv2.destroyAllWindows()
            finally:
                for sig, old in previous.items():
                    signal.signal(sig, old)
        if last_snapshot_event is not None:
            try:
                save_snapshot(folder, last_snapshot_frame, last_snapshot_debug,
                              last_snapshot_event, ['RUN_END'], args.drive,
                              suffix='END', final_status=status)
            except Exception as snapshot_error:
                print('[snapshot error]', snapshot_error, flush=True)
        _finish_logs(folder, status, frames)
    return 1 if ctl.policy.state == 'HALT' else 0


if __name__ == '__main__':
    raise SystemExit(main())

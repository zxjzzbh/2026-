#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""独立运行锥桶避障，并提供比赛入口共用的电机和转向接口。

直接运行时默认只观察，使用 --drive 并输入 MOVE 后才允许驱动。
观察和实车模式都会恢复云台姿态、读取相机并保存调试记录。
本文件与 config.py、camera_stream.py、camera_pose.json 放在同一目录。
"""

import argparse
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import signal
import threading
import time

import cv2
from cone_solo_logic import Params, Avoider, draw_debug

VERSION = "1.4-ultra-slow-ramp"
# 电机中位保持原标定：STOP_DUTY=12000，200Hz/range=40000。
# 当前先以超低速调避障：取消起步冲量，从STOP缓慢爬到低速巡航。
START_DUTY = 12600
START_TIME = 0.0
CRUISE_DUTY = 12600
MAX_FORWARD_DUTY = 16400
# 不使用起步冲量；从 STOP=12000 缓慢爬到 12600。
MOTOR_SLEW_PER_SECOND = 600.0


def parse_args(argv=None):
    """读取独立绕桶的命令参数，并检查运行时长是否在允许范围内。"""
    p = argparse.ArgumentParser(description="独立锥桶避障；默认观察，不使用白线")
    p.add_argument(
        "--drive", action="store_true", help="实车模式；启动后输入 MOVE 确认"
    )
    p.add_argument("--show", action="store_true", help="显示OpenCV窗口，需要X11/桌面")
    p.add_argument("--seconds", type=float, help="总时长；默认观察30秒，实车0=持续运行")
    p.add_argument("--color", choices=["blue", "red"], default="blue")
    p.add_argument(
        "--front-x", type=float, default=0.47, help="临时前进方向x/图宽，默认0.47"
    )
    p.add_argument("--turn-us", type=float, default=90, help="转向偏移上限，默认90微秒")
    p.add_argument(
        "--pass-seconds",
        type=float,
        default=0.60,
        help="绕过后保持通过时间，默认0.60秒",
    )
    p.add_argument(
        "--align-seconds", type=float, default=0.30, help="短暂反打时间，默认0.30秒"
    )
    p.add_argument(
        "--trigger-height",
        type=float,
        default=6,
        help="触发高度，按288像素图高折算；不是距离",
    )
    a = p.parse_args(argv)
    if a.seconds is None:
        a.seconds = 0.0 if a.drive else 30.0
    if not math.isfinite(a.seconds) or not (0 <= a.seconds <= 600):
        p.error("时长须为0至600秒；0表示持续运行")
    return a


def steering_pulse(c, turn):
    """把绕桶转向偏移换成舵机脉宽，并限制在已标定的转向范围内。"""
    return int(
        max(
            c.STEER_MIN_US, min(c.STEER_MAX_US, c.STEER_CENTER_US + c.STEER_SIGN * turn)
        )
    )


def checked(value):
    """检查 GPIO 调用结果；遇到负错误码就报错，否则原样返回结果。"""
    if isinstance(value, (int, float)) and value < 0:
        raise RuntimeError("pigpio 返回错误码：%s" % value)
    return value


class CarIO:
    """按配置控制电机和转向，初始化先关使能，arm 后才允许前进。

    独立线程监控新控制命令。它不等同于硬件急停，不能覆盖系统宕机/
    pigpiod失联/其他进程争抢GPIO；现场仍需有人可直接断驱动电源。
    """

    def __init__(self, c, pi=None, clock=time.monotonic, start_watchdog=True):
        """校验引脚和电调参数，保持停车输出，再按需启动超时监视线程。"""
        self.c, self.clock = c, clock
        self.lock = threading.RLock()
        self.closed = threading.Event()
        self.enabled, self.reason = False, ""
        self.last_command = clock()
        self.current = c.STOP_DUTY
        self.peak = c.STOP_DUTY
        self.ever_armed = False
        self.armed_at = None
        self.thread = None
        if pi is None:
            import pigpio

            pi = pigpio.pi()
        self.pi = pi
        if not pi.connected:
            raise RuntimeError("pigpiod未连接，请先启动sudo pigpiod")
        try:
            if (
                len(
                    {
                        c.ENABLE_GPIO,
                        c.MOTOR_GPIO,
                        c.STEER_GPIO,
                        c.YAW_GPIO,
                        c.PITCH_GPIO,
                    }
                )
                != 5
            ):
                raise ValueError("GPIO发生重复，请检查config.py")
            if (c.MOTOR_PWM_FREQ, c.MOTOR_PWM_RANGE, c.STOP_DUTY) != (
                200,
                40000,
                12000,
            ):
                raise ValueError("PWM参数与已提供的电调配置不同，拒绝套用本测试")
            checked(pi.set_mode(c.ENABLE_GPIO, 1))
            checked(pi.write(c.ENABLE_GPIO, 0))
            checked(pi.set_mode(c.MOTOR_GPIO, 1))
            checked(pi.set_PWM_frequency(c.MOTOR_GPIO, c.MOTOR_PWM_FREQ))
            checked(pi.set_PWM_range(c.MOTOR_GPIO, c.MOTOR_PWM_RANGE))
            checked(pi.set_PWM_dutycycle(c.MOTOR_GPIO, c.STOP_DUTY))
            checked(pi.set_mode(c.STEER_GPIO, 1))
            checked(pi.set_servo_pulsewidth(c.STEER_GPIO, c.STEER_CENTER_US))
            if (
                pi.get_PWM_frequency(c.MOTOR_GPIO) != c.MOTOR_PWM_FREQ
                or pi.get_PWM_range(c.MOTOR_GPIO) != c.MOTOR_PWM_RANGE
            ):
                raise RuntimeError("pigpio实际PWM频率/range不符")
        except BaseException:
            self.stop("SETUP_FAILED")
            pi.stop()
            raise
        if start_watchdog:
            self.thread = threading.Thread(target=self._watch, daemon=True)
            self.thread.start()

    def arm(self):
        """打开驱动使能并记录起步时间；已有停车原因时拒绝重新发车。"""
        with self.lock:
            if self.reason or self.closed.is_set():
                raise RuntimeError("停车已锁定，须退出检查后重新运行")
            self.last_command = self.clock()
            checked(self.pi.write(self.c.ENABLE_GPIO, 1))
            self.enabled = True
            self.ever_armed = True
            self.armed_at = self.clock()

    def apply(self, pulse, duty, captured_at):
        """检查画面时效与控制范围，再按原有加减速限制输出转向和动力。"""
        with self.lock:
            if self.reason or not self.enabled:
                raise RuntimeError("未使能或已停车，拒绝运动命令")
            now = self.clock()
            if not math.isfinite(captured_at) or not (0 <= now - captured_at <= 0.25):
                self.stop("STALE_COMMAND")
                raise RuntimeError("处理后的图像已经过期，停止驱动")
            if not (self.c.STOP_DUTY <= duty <= MAX_FORWARD_DUTY) or not math.isfinite(
                pulse
            ):
                self.stop("INVALID_COMMAND")
                raise ValueError("电机指令须在STOP_DUTY至16400之间，转向值须为有限数")
            pulse = max(self.c.STEER_MIN_US, min(self.c.STEER_MAX_US, int(pulse)))
            # 超低速调试：START_TIME=0 时不使用起步冲量，只按斜坡升到巡航档。
            launch = self.armed_at is not None and (now - self.armed_at) < START_TIME
            requested = START_DUTY if launch else duty
            if launch:
                target = float(START_DUTY)
            else:
                dt = max(0.0, min(0.10, now - self.last_command))
                step = MOTOR_SLEW_PER_SECOND * dt
                target = self.current + max(-step, min(step, requested - self.current))
            try:
                checked(self.pi.set_servo_pulsewidth(self.c.STEER_GPIO, pulse))
                checked(self.pi.set_PWM_dutycycle(self.c.MOTOR_GPIO, int(target)))
            except BaseException:
                self.stop("GPIO_ERROR")
                raise
            self.current = int(target)
            self.peak = max(self.peak, self.current)
            self.last_command = now

    def stop(self, reason="STOP"):
        """锁定首次停车原因，依次尝试电机回中、关闭使能和方向回中。"""
        with self.lock:
            self.reason = self.reason or reason
            self.enabled = False
            # 独立尝试每一步；某次写失败仍继续尝试关闭使能。
            for fn, args in (
                (self.pi.set_PWM_dutycycle, (self.c.MOTOR_GPIO, self.c.STOP_DUTY)),
                (self.pi.write, (self.c.ENABLE_GPIO, 0)),
                (
                    self.pi.set_servo_pulsewidth,
                    (self.c.STEER_GPIO, self.c.STEER_CENTER_US),
                ),
            ):
                try:
                    checked(fn(*args))
                except Exception as e:
                    print("[stop write failed]", e, flush=True)
            self.current = self.c.STOP_DUTY

    def check_watchdog(self):
        """检查运动指令是否超过 0.40 秒未更新，超时就锁定停车。"""
        with self.lock:
            if self.enabled and self.clock() - self.last_command > 0.40:
                self.stop("WATCHDOG")
                return True
        return False

    def _watch(self):
        """后台定期检查指令超时，收到关闭通知后结束。"""
        while not self.closed.wait(0.025):
            self.check_watchdog()

    def close(self):
        """先停车，再结束超时监视线程并断开 GPIO 连接。"""
        self.stop("EXIT")
        self.closed.set()
        if self.thread is not None:
            self.thread.join(timeout=1.0)
        self.pi.stop()


def restore_gimbal(c):
    """核对保存的云台姿态与相机配置，恢复两个舵机位置并等待稳定。"""
    import pigpio

    pose = json.loads(c.CAMERA_POSE_FILE.read_text(encoding="utf-8"))
    for key, expected in (
        ("yaw_gpio", c.YAW_GPIO),
        ("pitch_gpio", c.PITCH_GPIO),
        ("frame_width", c.CAMERA_WIDTH),
        ("frame_height", c.CAMERA_HEIGHT),
    ):
        if int(pose[key]) != expected:
            raise ValueError("camera_pose.json与config.py不一致：" + key)
    values = [(c.YAW_GPIO, int(pose["yaw_us"])), (c.PITCH_GPIO, int(pose["pitch_us"]))]
    if any(not 500 <= v <= 2500 for _, v in values):
        raise ValueError("保存的云台脉宽无效")
    pi = pigpio.pi()
    if not pi.connected:
        raise RuntimeError("pigpiod未连接")
    try:
        for pin, value in values:
            checked(pi.set_servo_pulsewidth(pin, value))
            time.sleep(0.3)
        time.sleep(c.GIMBAL_SETTLE_SECONDS)
    finally:
        pi.stop()
    return pose


def save_image(path, image):
    """保存一张调试图；写入失败时抛出异常，让主循环进入停车收尾。"""
    if not cv2.imwrite(str(path), image):
        raise OSError("图片保存失败：" + str(path))


def main(argv=None):
    """运行独立观察或绕桶流程，逐帧记录结果，退出时先停驱动再清理。"""
    args = parse_args(argv)
    params = Params(
        color=args.color,
        front_x=args.front_x,
        max_turn_us=args.turn_us,
        pass_seconds=args.pass_seconds,
        align_seconds=args.align_seconds,
        trigger_height=args.trigger_height,
    )
    if args.show and not os.environ.get("DISPLAY"):
        raise RuntimeError("没有DISPLAY；去掉--show使用终端+自动保存图片")
    # 非实时模块导入不会碰GPIO；硬件只在main里按模式显式创建。
    import config as C
    from camera_stream import LatestFrameCamera

    duty = CRUISE_DUTY
    if not C.STOP_DUTY < duty <= MAX_FORWARD_DUTY:
        raise ValueError("CRUISE_DUTY必须大于STOP_DUTY且不超过16400")
    debug_dir = Path(__file__).resolve().parent / "cone_debug"
    debug_dir.mkdir(exist_ok=True)
    if args.drive:
        print("实车：先架空确认驱动与停止；地面测试清场，准备直接断驱动电源。")
        print(
            "[motor] version=%s start=%d x %.2fs -> cruise=%d stop=%d freq=%d range=%d"
            % (
                VERSION,
                START_DUTY,
                START_TIME,
                duty,
                C.STOP_DUTY,
                C.MOTOR_PWM_FREQ,
                C.MOTOR_PWM_RANGE,
            )
        )
        print("只避蓝/红色候选，不避人或其他颜色物体；没有白线边界和测距保证。")
        print("[mode] 持续绕行：底边位置不测距、不触发停车；反打/保持后继续走。")
        print(
            "[timing] align=%.2fs pass=%.2fs；未经实车标定，不保证车尾通过或回原路线。"
            % (params.align_seconds, params.pass_seconds)
        )
        print(
            "[limit] "
            + (
                "无定时结束，Ctrl+C退出。"
                if args.seconds == 0
                else "%.1f秒后结束测试并停机。" % args.seconds
            )
        )
        if input("确认允许电机运动，输入大写 MOVE：").strip() != "MOVE":
            print("未确认，退出。")
            return 0
    else:
        print("观察模式：先断驱动电源，保留树莓派、相机、云台供电。")
        input("确认其他车控程序已退出，按回车开始：")
    car, camera = None, None
    status = "STARTING"
    frames = 0
    metadata = dict(
        args=vars(args),
        params=asdict(params),
        version=VERSION,
        motor=dict(
            requested_duty=duty,
            stop_duty=C.STOP_DUTY,
            frequency=C.MOTOR_PWM_FREQ,
            pwm_range=C.MOTOR_PWM_RANGE,
            ceiling=MAX_FORWARD_DUTY,
            slew_per_second=MOTOR_SLEW_PER_SECOND,
            start_duty=START_DUTY,
            start_time=START_TIME,
            cruise_duty=CRUISE_DUTY,
        ),
        note="Continuous bypass. Pixel bottom is not distance. Timed alignment/pass are uncalibrated. PWM is not measured speed.",
        distance_model="none",
        timer_enabled=bool(args.seconds),
    )

    def interrupt(signum, frame):
        """把系统退出信号转成中断，让主流程执行统一的停车收尾。"""
        raise KeyboardInterrupt

    previous = {s: signal.signal(s, interrupt) for s in (signal.SIGTERM, signal.SIGHUP)}
    try:
        if args.drive:
            car = CarIO(C)  # 先保持中位与使能关闭
            metadata["motor"].update(
                actual_frequency=car.pi.get_PWM_frequency(C.MOTOR_GPIO),
                actual_range=car.pi.get_PWM_range(C.MOTOR_GPIO),
            )
        metadata["camera_pose"] = restore_gimbal(C)
        camera = LatestFrameCamera()  # 先赋值；start失败仍可在finally释放
        camera.start()
        logic = Avoider(params)
        start = time.monotonic()
        last_id = -1
        next_save, next_print = start, start
        last_state = None
        with (debug_dir / "events.jsonl").open("w", encoding="utf-8") as log:
            while args.seconds == 0 or time.monotonic() - start < args.seconds:
                # 先检查停车锁、画面时效和帧号，避免使用旧画面控制车辆。
                now = time.monotonic()
                frame, fid, ts = camera.read()
                if car is not None and car.reason:
                    status = car.reason
                    break
                if frame is None or now - ts > params.max_age:
                    status = "STALE_FRAME"
                    if car:
                        car.stop(status)
                    break
                if fid == last_id:
                    time.sleep(0.003)
                    continue
                last_id = fid
                # 识别与避障判断完成后再次检查耗时，再决定是否输出驱动。
                r = logic.update(frame, fid, ts, now)
                frames += 1
                status = r.state + ":" + r.reason
                pulse = steering_pulse(C, r.turn_us)
                if time.monotonic() - ts > params.max_age:
                    r = logic.halt("PROCESSING_TOO_SLOW", r.boxes, r.mask)
                    status = r.reason
                if car:
                    if r.stop:
                        car.stop(r.reason or r.state)
                    elif frames >= 5:  # 五帧健康检查；候选触底本身不拒绝起步
                        if not car.enabled:
                            car.arm()
                            print(
                                "[drive] PRECHECK_OK -> ENABLE=1 target=%d" % duty,
                                flush=True,
                            )
                        car.apply(pulse, duty, ts)
                # 日志阶段：观察、起步检查、行驶或已停车。
                phase = (
                    "OBSERVE"
                    if car is None
                    else (
                        ("PRECHECK_BLOCKED" if r.stop else "PRECHECK")
                        if not car.ever_armed
                        else ("HALT" if r.stop else "DRIVE")
                    )
                )
                cropped = [
                    dict(id=b["id"], bottom=b["bottom"])
                    for b in r.boxes
                    if b["clipped"]
                ]
                event = dict(
                    frame=fid,
                    t=now - start,
                    captured_at=ts,
                    state=r.state,
                    reason=r.reason,
                    stop=r.stop,
                    turn_us=r.turn_us,
                    phase=phase,
                    steer_us=pulse,
                    motor_duty=(car.current if car else None),
                    requested_duty=duty,
                    enable_command=(int(car.enabled) if car else None),
                    cropped_candidates=cropped,
                    distance_model="none",
                    boxes=r.boxes,
                )
                log.write(json.dumps(event, ensure_ascii=False) + "\n")
                state_key = (r.state, r.reason, r.side)
                if now >= next_print or state_key != last_state:
                    print(
                        "f=%d phase=%s state=%s reason=%s enable=%s target=%d motor=%s turn=%+.0f steer=%d"
                        % (
                            fid,
                            phase,
                            r.state,
                            r.reason or "-",
                            "-" if car is None else int(car.enabled),
                            duty,
                            "OFF" if car is None else int(car.current),
                            r.turn_us,
                            pulse,
                        ),
                        flush=True,
                    )
                    next_print = now + 0.25
                    last_state = state_key
                if now >= next_save or (car and r.stop):
                    save_image(debug_dir / "latest_raw.jpg", frame)
                    save_image(
                        debug_dir / "latest_debug.jpg", draw_debug(frame, r, params)
                    )
                    if r.mask is not None:
                        save_image(debug_dir / "latest_mask.png", r.mask)
                    log.flush()
                    next_save = now + 0.5
                if args.show:
                    cv2.imshow("Cone only", draw_debug(frame, r, params))
                    key = cv2.waitKey(1) & 255
                    if key in (27, ord("q")):
                        status = "USER_QUIT"
                        break
                    if key == ord("r") and not args.drive:
                        # 只允许观察模式人工重置；实车停车不自恢复。
                        logic = Avoider(params)
                if car and r.stop:
                    break
            else:
                status = "TIME_LIMIT"
    except KeyboardInterrupt:
        status = "USER_INTERRUPT"
        print("\n收到退出信号。")
    except Exception as e:
        status = "ERROR: " + repr(e)
        raise
    finally:
        # 先停止驱动，再关闭摄像头、窗口并保存总结，避免清理时继续运动。
        if car is not None:
            car.close()
        if camera is not None:
            try:
                camera.stop()
            except Exception as e:
                print("[camera close]", e)
        if args.show:
            cv2.destroyAllWindows()
        for s, handler in previous.items():
            signal.signal(s, handler)
        metadata.update(
            status=status,
            processed_frames=frames,
            ever_armed=(car.ever_armed if car else False),
            peak_commanded_duty=(car.peak if car else None),
            actuator_stop_reason=(car.reason if car else None),
        )
        (debug_dir / "run.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print("\n状态：%s\n调试文件：%s" % (status, debug_dir), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

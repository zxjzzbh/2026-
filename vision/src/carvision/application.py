"""Explicit vehicle composition: one camera pair, one manual owner, one mode.

Competition imports no debug panel or launcher. Debug task adapters call the
same task cores and use a flat action registry, rather than wrapping launchers.
"""
import argparse
import copy
import json
from pathlib import Path
import signal
import sys
import threading
from types import SimpleNamespace
from urllib.parse import parse_qs

from .boot_test import BootTestDrive, PiBootPreparation
from .config import load_config
from .manual_drive import PausedDrive
from .payment_console import PaymentConsoleDrive, attach_payment_capture
from .race_flow_console import RaceFlowDrive
from .web_preview import PreviewState, PreviewWorker, make_server


def _finish_all(actions):
    """An adapter failure cannot skip another owner's stop or cleanup."""
    errors=[]
    for name,finish in actions:
        try:finish()
        except Exception as exc:errors.append({'component':name,'error':str(exc)})
    return errors


class VehicleBootDrive(BootTestDrive):
    def status(self):
        return {**super().status(), 'esc_mode': 'F/B', 'reverse_available': False,
                'manual_reverse_disabled_reason': '当前 F/B 档没有倒车功能'}

    def request(self, action, payload):
        if action == 'command' and isinstance(payload, dict) and payload.get('motor') == 'reverse':
            super().request('stop', {'client': payload.get('client'), 'stop_source': 'fb_reverse_rejected'})
            raise ValueError('当前 F/B 档禁止倒车；未发送倒车或刹车替代指令')
        return super().request(action, payload)

    def borrow_actuator(self, settings, client):
        """Borrow the explicitly constructed manual worker; keep its UART owner."""
        from .shared_executor import BorrowedActuator
        with self.lock:
            if self.phase != 'ready' or self.delegate is None:
                raise ValueError('先完成遥控准备并在切换区停稳')
            deferred = self.delegate
            with deferred.lock:
                worker = deferred.delegate
            if worker is None:
                raise ValueError('遥控执行器尚未就绪，请等待本页遥控启用完成')
            state = worker.status()
            if state.get('mode') != 'enabled':
                raise ValueError('请先在本页启用遥控，等待遥控已就绪并停稳后再切入自主驾驶')
            if state.get('owner_client') != client:
                raise ValueError('本页未持有已就绪的遥控执行器')
            return BorrowedActuator(worker, settings, client)


def make_boot_drive(root, *, allow_handoff):
    from .manual_service import repeated_drive, initial_status
    template_path = root / 'vision/deployment/boot-test-template.json'
    template = json.loads(template_path.read_text(encoding='utf-8-sig'))
    preparation = PiBootPreparation(root, template_path)
    profiles = {'driving': initial_status(template)}
    if not allow_handoff:
        profiles['steering_pwm'] = initial_status({**template, 'steering_pwm_mode': True,
                                                  'ground_continuous_requested_by_user': False})
    boot = VehicleBootDrive(
        lambda cancelled: preparation.prepare(cancelled, mode=boot.selected_mode),
        preparation.confirm,
        lambda context: repeated_drive(root, context, allow_autonomy_handoff=allow_handoff),
        profiles['driving'], profiles=profiles)
    preparation.start_monitor()
    return boot, preparation


class BaseView:
    """Task-local base view; closing an adapter cannot close another task's base."""
    def __init__(self, application):
        self.application = application

    def status(self):
        return {**self.application.manual.status(), **dict(self.application.extensions)}

    def request(self, action, payload):
        return self.application.manual.request(action, payload)

    def close(self):
        pass


def active_owners(state):
    rows = [state.get(key) or {} for key in
            ('crosswalk_trial', 'traffic_trial', 'departure_trial', 'parking_trial')]
    cone = state.get('cone_trial') or {}
    rows += [cone.get('avoidance') or {}, cone.get('short_test') or {}]
    payment = state.get('parking_payment') or {}
    if payment.get('motion_locked'):
        rows.append({**payment, 'active': True})
    return [row for row in rows if row.get('active')]


class DebugApplication:
    """Flat task registry with one global movement/ownership gate."""
    def __init__(self, manual, states, root):
        from .brake_trial import BrakeTrialDrive
        from .traffic_trial import TrafficTrialDrive
        from .cone_observer import ConeObserverDrive
        from .departure_trial import DepartureTrialDrive
        from .parking_bay_console import ParkingBayDrive
        self.manual, self.root = manual, root
        self.lock = threading.RLock()
        self.extensions = {}
        self.closed=False;self.stop_errors=[];self.close_errors=[]
        view = BaseView(self)
        configs = root / 'vision/configs'
        self.tasks = {
            'brake': BrakeTrialDrive(view, states, configs/'brake-parking.json', root),
            'traffic': TrafficTrialDrive(view, states, configs/'traffic-driving.json', configs/'traffic-signal.json', root),
            'cones': ConeObserverDrive(view, states, configs/'cone-trial.json', root),
            'departure': DepartureTrialDrive(view, states, root),
            'parking': ParkingBayDrive(view, states, root),
            'payment': PaymentConsoleDrive(view, states, root),
        }
        self.prefixes = {'crosswalk_': 'brake', 'traffic_': 'traffic', 'cone_': 'cones',
                         'departure_': 'departure', 'parking_bay_': 'parking',
                         'parking_trial_': 'parking', 'payment_': 'payment'}

    @staticmethod
    def task_active(key, state):
        if key == 'cones':
            row = state.get('cone_trial') or {}
            return any((row.get(name) or {}).get('active') for name in ('avoidance', 'short_test'))
        field = {'brake': 'crosswalk_trial', 'traffic': 'traffic_trial',
                 'departure': 'departure_trial', 'parking': 'parking_trial',
                 'payment': 'parking_payment'}[key]
        row = state.get(field) or {}
        return bool(row.get('motion_locked') if key == 'payment' else row.get('active'))

    @property
    def task_settings(self):
        return {key: copy.deepcopy(node.settings) for key, node in self.tasks.items()
                if key != 'payment' and hasattr(node, 'settings')}

    def status(self):
        base = self.manual.status()
        for node in self.tasks.values():
            result = node.status()
            self.extensions.update({k: v for k, v in result.items() if k not in base})
        return {**base, **self.extensions, 'application_mode': 'debug',
                'manual_owner_client': base.get('owner_client'),
                'stop_errors':list(self.stop_errors),'close_errors':list(self.close_errors)}

    def request(self, action, payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        if action == 'status':
            return self.status()
        with self.lock:
            if self.closed:raise RuntimeError('调试服务已关闭')
            state = self.status()
            active = active_owners(state)
            owner = next((row.get('owner_client') for row in active if row.get('owner_client')), state.get('owner_client'))
            if action in ('stop', 'finish_test') and owner and payload.get('client') != owner:
                return state
            if action in ('stop', 'emergency', 'finish_test'):
                stop_action='emergency' if action=='emergency' else 'stop'
                actions=[]
                if action != 'finish_test' or state.get('boot_test_phase') == 'ready':
                    actions.append(('manual',lambda:self.manual.request(action,payload)))
                actions.extend((key,lambda node=node:node.request(stop_action,payload))
                               for key,node in self.tasks.items())
                self.stop_errors=_finish_all(actions)
                return self.status()
            target_key = next((key for prefix, key in self.prefixes.items() if action.startswith(prefix)), None)
            if target_key is not None:
                # A task's own start/heartbeat remains valid, but preparing a
                # second movement task cannot acquire the same hardware.
                other_active = any(self.task_active(key, state) for key in self.tasks if key != target_key)
                if other_active and action.endswith(('_prepare', '_start')):
                    raise ValueError('另一单项任务占用控制，请先结束该任务')
                self.tasks[target_key].request(action, payload)
            else:
                if action.startswith('race_'):
                    raise ValueError('调试入口不运行完整比赛，请使用桌面比赛脚本')
                if active:
                    raise ValueError('单项任务占用控制，请先结束该任务')
                self.manual.request(action, payload)
            return self.status()

    def close(self):
        with self.lock:
            if self.closed:return
            self.closed=True
        actions=[(key,node.close) for key,node in reversed(tuple(self.tasks.items()))]
        actions.append(('manual',self.manual.close))
        self.close_errors=_finish_all(actions)


class CompetitionServices:
    def __init__(self, manual, states, root):
        self.manual = manual
        self.payment = PaymentConsoleDrive(manual, states, root)
        self.closed=False;self.close_errors=[]

    def status(self):
        state = self.payment.status()
        return {**state, 'application_mode': 'race', 'manual_owner_client': state.get('owner_client')}

    def request(self, action, payload):
        state = self.status()
        if action in ('stop', 'finish_test') and state.get('owner_client') and payload.get('client') != state['owner_client']:
            return state
        return self.payment.request(action, payload)

    def borrow_actuator(self, settings, client):
        return self.manual.borrow_actuator(settings, client)

    def close(self):
        if self.closed:return
        self.closed=True
        # PaymentConsoleDrive only owns its scanner, never the manual worker.
        self.close_errors=_finish_all([('payment',self.payment.close),('manual',self.manual.close)])


def image_response(data):
    return (data, 'image/jpeg', 200) if data else (b'No fresh task frame', 'text/plain', 503)


def register_images(server, application, *, mode):
    payment = application.drive.payment if mode == 'race' else application.tasks['payment']
    server.routes['/api/payment/image.jpg'] = lambda url: image_response(payment.image(parse_qs(url.query).get('id', [''])[0]))
    if mode == 'race':
        def race_image(_):
            with application.lock:
                fresh = application.updated is not None and 0 <= application.clock()-application.updated <= .25
                return image_response(application.preview if fresh else None)
        server.routes['/api/race-flow/frame.jpg'] = race_image
        return
    for path, key in (('/api/departure/frame.jpg', 'departure'), ('/api/parking-bay/frame.jpg', 'parking')):
        server.routes[path] = lambda _, key=key: image_response(application.tasks[key].image())
    def traffic_image(_):
        task = application.tasks['traffic']
        with task.lock:
            fresh = task.busy and 0 <= task.clock()-(task.observation or {}).get('captured_s', -1e9) <= task.settings['max_frame_age_s']
            return image_response(task.preview_jpeg if fresh else None)
    def cone_image(_):
        task = application.tasks['cones']
        with task.lock:
            fresh = task.observing and task.updated is not None and 0 <= task.clock()-task.updated <= task.settings['max_frame_age_s']
            return image_response(task.preview if fresh else None)
    server.routes.update({'/api/traffic/frame.jpg': traffic_image, '/api/cone/frame.jpg': cone_image})


def main(mode, argv=None):
    parser = argparse.ArgumentParser(description='Independent '+mode+' vehicle service')
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--live', action='store_true')
    args = parser.parse_args(argv)
    root = args.root.resolve()
    cameras = json.loads((root/'vision/configs/cameras.json').read_text(encoding='utf-8-sig'))
    states = {key: PreviewState(1500) for key in ('primary', 'secondary')}
    attach_payment_capture(states['secondary'])
    preparation = None
    services = None
    if args.live:
        manual, preparation = make_boot_drive(root, allow_handoff=mode == 'race')
    else:
        manual = PausedDrive('只读预览；实车服务需要 --live 和现场准备')
    if mode == 'race':
        services = CompetitionServices(manual, states, root)
        application = RaceFlowDrive(services, states, root, live=args.live, require_ready=True)
    elif mode == 'debug':
        application = DebugApplication(manual, states, root)
    else:
        raise ValueError('Unknown application mode')
    server = make_server((args.host, args.port), states['primary'], states['secondary'], application, build_controls=False)
    register_images(server, application, mode=mode)
    from .competition_page import add_page
    add_page(server, root=root, at_root=mode == 'race', extra_paths=('/manual',))
    if mode == 'debug':
        from .debug_page import add_page as add_debug_page
        add_debug_page(server, root=root, at_root=True)
    config = load_config(None)
    workers = []
    for camera, state in states.items():
        options = SimpleNamespace(camera=cameras[camera], width=640, height=480, fps=15,
            preview_fps=8, raw_preview_fps=15, preview_width=480, jpeg_quality=55,
            stale_ms=1500, camera_format='MJPG', weights=None, classic_candidates=False,
            race_config=str(root/'vision/configs/race-2026.json'))
        workers.append(PreviewWorker(state, config, options))
    def finish(*_):
        raise KeyboardInterrupt
    for name in ('SIGTERM', 'SIGINT', 'SIGHUP'):
        if hasattr(signal, name):signal.signal(getattr(signal, name), finish)
    try:
        for worker in workers:worker.start()
        print(mode+' service ready; hardware remains off until explicit preparation', flush=True)
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        actions=[('application',application.close)]
        # RaceFlowDrive only releases the borrowed autonomous lease. The app
        # also owns the original manual/payment service and must close it.
        if services is not None:actions.append(('services',services.close))
        if preparation:actions.append(('preparation',preparation.close))
        actions.extend(('camera_'+str(index),worker.close) for index,worker in enumerate(workers))
        actions.append(('server',server.server_close))
        errors=_finish_all(actions)
        for name,resource in (('application',application),('services',services)):
            errors.extend({'component':name+'/'+row['component'],'error':row['error']}
                          for row in getattr(resource,'close_errors',[]))
        for error in errors:print('Cleanup '+error['component']+': '+error['error'],file=sys.stderr,flush=True)

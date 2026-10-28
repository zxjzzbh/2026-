"""Explicit manual/autonomous ownership transfer over one already-open backend.

The autonomous lease and finite F/B pulses remain enforced in the hardware
worker. Borrowing never opens a second UART owner or verifies a physical stop.
"""
import secrets
import threading
import time

from .cone_trial_output import ConePulseGuard


INTENT_FIELDS=frozenset(('action','esc_us','steering_us','sequence','sent_s'))


class WorkerAutonomyOwner:
    def __init__(self,controller,backend,enabled=False,clock=time.monotonic):
        self.controller,self.backend,self.enabled,self.clock=controller,backend,enabled,clock
        self.lock=threading.RLock()
        self.guard=None;self.owner=self.token=None
        self.reason='尚未借用执行器';self.closed=False;self.resume_watcher=False
        self.handoff_fault=None
        self.last_motion_at=None

    @property
    def active(self):return self.token is not None

    def status(self):
        # Observation never renews the autonomous lease or writes output.
        with self.lock:
            guard=self.guard
            return {**self.controller.status(),'owner_client':self.controller.owner,
                    'autonomy_handoff_enabled':bool(self.enabled),
                    'autonomy_active':self.active,'autonomy_owner_client':self.owner,
                    'autonomy_draining':guard is not None and not self.active,
                    'autonomy_reason':self.reason,'autonomy_fault':guard.fault if guard else self.handoff_fault,
                    'autonomy_handoff_fault':self.handoff_fault,
                    'autonomy_sequence':guard.sequence if guard else None,
                    'autonomy_esc_us':guard.esc if guard else 1500,
                    'autonomy_steering_us':guard.actual_steering if guard else 1610,
                    'physical_stop_verified':False}

    def _write(self,channel,pulse):
        if channel==3:self.backend.steering(pulse)
        elif pulse==1500:self.backend.neutral();self.last_motion_at=None
        else:
            guard=self.guard;now=self.clock()
            deadline=(guard.emergency_until or guard.brake_until or guard.deadline) if guard else None
            # PulseGuard.abort writes its first brake edge before assigning
            # emergency_until; its existing forward lease is already retired.
            if guard and guard.fault and guard.esc>1500:
                deadline=now+guard.s['brake_pulse_s']
            self._motion_segment(pulse,deadline)

    def _motion_segment(self,pulse,deadline):
        now=self.clock();remaining=deadline-now if deadline is not None else 0.
        if remaining<.02:
            # The existing Pi guardian rejects segments below 20 ms. Leave the
            # prior bounded segment to finish; neither restart nor extend it.
            return
        else:
            self.backend.motion(pulse,min(.2,remaining));self.last_motion_at=now

    def _pause_manual(self):
        # ManualDrive's own watcher would otherwise recenter/idle S3 during an
        # autonomous turn. Stop that watcher, leaving the Pi guardian running.
        self.controller.emergency('执行器已交给自主流程；遥控需重新启用')
        self.controller.stop_event.set()
        thread=self.controller.thread
        self.resume_watcher=bool(thread and thread.is_alive())
        if thread and thread is not threading.current_thread():
            thread.join(timeout=1)
            if thread.is_alive():raise RuntimeError('遥控检查线程尚未释放，不能接管执行器')

    def _resume_manual(self):
        # Mode remains emergency: resuming safety checks cannot replay keys.
        if self.resume_watcher and not self.closed and self.handoff_fault is None:
            thread=self.controller.thread
            if thread and thread.is_alive():
                self._latch_handoff_failure('旧遥控检查线程尚未退出')
                raise RuntimeError(self.handoff_fault)
            self.controller.stop_event.clear()
            self.controller.start()
        self.resume_watcher=False

    def _latch_handoff_failure(self,reason):
        # A partially stopped manual watcher must never be reset/re-enabled
        # without its safety loop. Retire this worker until fresh preparation,
        # including when a timed-out old thread eventually exits much later.
        self.handoff_fault='共享接管失败，必须重新准备工作进程：'+str(reason)
        self.reason=self.handoff_fault;self.guard=None;self.owner=self.token=None
        self.resume_watcher=False;self.controller.stop_event.set()
        with self.controller.lock:
            self.controller.mode='fault';self.controller.reason=self.handoff_fault
            self.controller.expires=None
        try:self.backend.neutral()
        except Exception:pass  # The original independent guardian remains bounded.

    def _authorized(self,payload):
        token=payload.get('worker_token')
        if (not self.active or payload.get('client')!=self.owner or not isinstance(token,str)
                or not secrets.compare_digest(token,self.token)):
            raise ValueError('自主执行器控制权已失效')

    def _acquire(self,payload):
        if self.handoff_fault is not None:raise ValueError(self.handoff_fault)
        if not self.enabled or self.closed:raise ValueError('当前工作进程未启用自主接管')
        if self.guard is not None:raise ValueError('执行器正在自主运行或完成制动')
        client=payload.get('client');settings=payload.get('settings')
        state=self.controller.status()
        if (not isinstance(client,str) or client!=state.get('owner_client',self.controller.owner)
                or self.controller.owner!=client or state.get('mode')!='enabled'):
            raise ValueError('仅当前已启用遥控的持有者可接管')
        if (payload.get('fb_mode_confirmed') is not True or state.get('motor_direction')!='stop'
                or state.get('motor_pulse_us')!=1500):
            raise ValueError('接管前保持 F/B 并先发送遥控停车')
        if (not isinstance(settings,dict) or getattr(self.backend,'steering_center_us',1610)!=1610
                or not getattr(self.backend,'motor_available',True)
                or not getattr(self.backend,'steering_available',True)):
            raise ValueError('当前执行器不能用于比赛接管')
        # Construct and validate before retiring manual ownership. Construction
        # performs no writes and cannot broaden the existing 1625-us envelope.
        guard=ConePulseGuard(settings,self._write,mode='F/B',
                             right_us=settings.get('right_us',1350),left_us=settings.get('left_us',1870))
        try:
            self._pause_manual()
            self.guard=guard;guard.neutral()
            self.owner=client;self.token=secrets.token_hex(24)
            self.reason='共享执行器已接管；物理停稳仍需相机确认'
        except Exception as exc:
            self._latch_handoff_failure(exc)
            raise
        return {**self.status(),'worker_token':self.token,'warm_handoff':True}

    def _begin_release(self,reason):
        # Invalidate before braking so a queued old intent/release cannot own a
        # subsequent run. The guard retains its original finite brake deadline.
        self.token=self.owner=None;self.reason=reason
        if self.guard is not None:self.guard.abort(reason,self.clock())

    def tick(self):
        with self.lock:
            guard=self.guard
            if guard is None:return
            self.backend.check()  # Preserve the original independent Pi guardian.
            now=self.clock();guard.tick(now)
            if guard.fault and self.active:self._begin_release(guard.fault)
            # The backend's guardian only receives finite <=200-ms segments.
            # Renew within the existing guard deadline, never past that lease
            # or the remaining finite brake interval.
            if guard.esc!=1500 and (self.last_motion_at is None or now-self.last_motion_at>=.09):
                deadline=guard.emergency_until or guard.brake_until or guard.deadline
                self._motion_segment(guard.esc,deadline)
            if not self.active and guard.emergency_until is None:
                guard.neutral();self.guard=None
                self._resume_manual()

    def dispatch(self,action,payload):
        if not isinstance(payload,dict):raise ValueError('JSON object required')
        with self.lock:
            if self.handoff_fault is not None and action not in ('status','stop','emergency','shutdown'):
                raise ValueError(self.handoff_fault)
            if action=='autonomy_acquire':return self._acquire(payload)
            if action in ('autonomy_intent','autonomy_release'):
                self._authorized(payload)
                if action=='autonomy_release':
                    self._begin_release('自主流程已释放执行器')
                    self.tick();return self.status()
                command=payload.get('intent')
                if not isinstance(command,dict) or set(command)!=INTENT_FIELDS:
                    self._begin_release('自主指令字段无效');self.tick()
                    raise ValueError('自主指令仅接受动作、脉宽、序号和发送时间')
                try:self.guard.accept(command,self.clock())
                except Exception:
                    self._begin_release('自主指令失效，完成有限制动后停车');self.tick()
                    raise
                return self.status()
            if action in ('stop','emergency','shutdown') and self.guard is not None:
                self._begin_release('全局停车请求：'+action);self.tick()
                return self.status()
            if action=='status':return self.status()
            if self.guard is not None:raise ValueError('自主流程持有执行器，遥控指令已拒绝')
            if action=='shutdown':self.controller.emergency('控制服务退出')
            else:self.controller.request(action,payload)
            return self.status()

    def close(self):
        with self.lock:
            self.closed=True;self._begin_release('共享执行器服务退出')
        # Bounded cleanup; the independent guardian also expires each segment.
        end=time.monotonic()+.6
        try:
            while self.guard is not None and time.monotonic()<end:
                self.tick()
                if self.guard is not None:time.sleep(.005)
        finally:
            with self.lock:
                if self.guard is not None:self.guard.neutral();self.guard=None
                self.owner=self.token=None


class BorrowedActuator:
    """Client of a live manual worker; close releases only this borrowed lease."""
    run=True

    def __init__(self,worker,settings,client,*,clock=time.monotonic):
        self.worker,self.client,self.clock=worker,client,clock
        self.sequence=0;self.closed=False;self.close_error=None
        reply=worker.request('autonomy_acquire',{'client':client,'settings':settings,'fb_mode_confirmed':True})
        token=reply.get('worker_token')
        if not reply.get('autonomy_active') or not isinstance(token,str):
            raise RuntimeError('共享执行器未确认接管')
        self.token=token
        self.startup_readback={'warm_handoff':True,'physical_stop_verified':False}

    def send(self,intent):
        if self.closed:raise RuntimeError('borrowed actuator closed')
        command={'action':intent['action'],'esc_us':intent['esc_us'],'steering_us':intent['steering_us'],
                 'sequence':self.sequence,'sent_s':self.clock()}
        self.sequence+=1
        reply=self.worker.request('autonomy_intent',{'client':self.client,'worker_token':self.token,'intent':command})
        if not reply.get('autonomy_active'):raise RuntimeError(reply.get('autonomy_reason') or '自主执行器已停车')
        return {'hardware_output':True,'command':command,'startup_readback':self.startup_readback,
                'tuning_verified':False}

    def close(self):
        if self.closed:return
        self.closed=True
        try:self.worker.request('autonomy_release',{'client':self.client,'worker_token':self.token})
        except (OSError,ValueError,RuntimeError) as exc:self.close_error=str(exc)

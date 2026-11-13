"""Independent fixed-PWM calibration sequencer; leaves existing drivers intact.

The child owns the *entire* timed sequence and checks a 200 ms parent lease.
Refreshing that lease never extends the forward interval. S1/S2 are untouched.
"""
import multiprocessing as mp
import sys
import time

from .brake_parking import numeric
from .steering import LEFT_US as STEERING_LEFT_US, CENTER_US as STEERING_CENTER_US, RIGHT_US as STEERING_RIGHT_US
from .brake_output import write_parking_pulse,verify_neutral_readback

MODES={'turn_1350':STEERING_RIGHT_US,'turn_1870':STEERING_LEFT_US,'brake':STEERING_CENTER_US}
PHASE_TEXT={'ready':'中位已准备','arming':'3 秒中位初始化','aligning':'固定转向，电机中位',
    'driving':'定时低速前进','braking':'F/B 主动制动','releasing':'刹车释放中位',
    'settling':'中位等待、随后回正','complete':'短测结束，请现场确认停稳','fault':'已停止短测'}


def validate_parameters(mode,forward_us,duration_s):
    if mode not in MODES:raise ValueError('只允许 1350、1870 固定转向或回中刹车短测')
    if type(forward_us) is not int or not 1501<=forward_us<=1625:raise ValueError('短测油门范围为 1501–1625 μs')
    if not numeric(duration_s) or not .2<=duration_s<=2.:raise ValueError('前进时长须为 0.2–2.0 秒')


class ShortGuard:
    def __init__(self,write,now):
        self.write=write;self.phase='ready';self.reason=PHASE_TEXT['ready'];self.fault=None
        self.sequence=-1;self.lease_until=now+2.;self.last_t=now  # neutral-only startup grace
        self.opened_s=now;self.round=0;self.mode=None;self.forward_us=1575;self.duration_s=.8
        self.steering_us=1610;self.esc_us=1500;self.deadline=None
        self.drive_started_s=self.brake_started_s=None;self.brake_count=0;self.last_brake_end=None
        self.ever_forward=False;self.emergency_start=self.emergency_until=None
        self._output(1500,1610)

    def _output(self,esc,steer):
        if steer not in MODES.values() or esc not in (1500,1400,self.forward_us):raise ValueError('invalid short-test output')
        # Remove drive first when braking/neutral; establish steering first
        # when entering forward. Alignment holds neutral without recentering.
        if esc>1500:self.write(3,steer);self.write(4,esc)
        else:self.write(4,esc);self.write(3,steer)
        self.esc_us,self.steering_us=esc,steer

    def _phase(self,phase,now,duration,esc,steer):
        self.phase=phase;self.reason=PHASE_TEXT[phase];self.deadline=now+duration if duration is not None else None
        self._output(esc,steer)

    def abort(self,reason,now):
        if self.fault:return
        old_phase=self.phase;self.fault=reason;self.reason=reason;self.phase='fault';self.lease_until=None
        if old_phase=='braking' and self.esc_us==1400:
            self.emergency_until=self.deadline  # never extend an existing pulse
        elif self.ever_forward and old_phase not in ('ready','complete'):
            self.emergency_start=max(now,(self.last_brake_end or -1e9)+.1)
            if self.emergency_start<=now:
                self._output(1400,self.steering_us);self.emergency_until=now+.12;self.emergency_start=None
        else:self._output(1500,1610)

    def tick(self,now):
        if not numeric(now):raise ValueError('invalid clock')
        if now<self.last_t:self.abort('时钟倒退',self.last_t);return
        self.last_t=now
        if not self.fault and now>=self.lease_until:self.abort('短测指令租约超时',now)
        if not self.fault and now-self.opened_s>=180:self.abort('短测会话超过 3 分钟',now)
        if self.fault:
            if self.emergency_start is not None and now>=self.emergency_start:
                self._output(1400,self.steering_us);self.emergency_until=now+.12;self.emergency_start=None
            if self.emergency_until is not None and now>=self.emergency_until:
                self.emergency_until=None;self._output(1500,1610)
            return
        if self.deadline is None or now<self.deadline:return
        steer=MODES[self.mode]
        if self.phase=='arming':self._phase('aligning',now,.4,1500,steer)
        elif self.phase=='aligning':
            self.drive_started_s=now;self.ever_forward=True
            self._phase('driving',now,self.duration_s,self.forward_us,steer)
        elif self.phase=='driving':
            self.brake_started_s=now;self.brake_count=1;self._phase('braking',now,.12,1400,steer)
        elif self.phase=='braking':
            self.last_brake_end=now
            self._phase('releasing' if self.brake_count<3 else 'settling',now,.1 if self.brake_count<3 else .5,1500,steer)
        elif self.phase=='releasing':
            self.brake_count+=1;self._phase('braking',now,.12,1400,steer)
        elif self.phase=='settling':self._phase('complete',now,None,1500,1610)

    def accept(self,message,now):
        self.tick(now)
        if self.fault:raise ValueError(self.fault)
        try:
            seq,t=message.get('sequence'),message.get('sent_s')
            if type(seq) is not int or seq<=self.sequence or not numeric(t) or not 0<=now-t<=.2:raise ValueError('stale or duplicate short-test command')
            action=message.get('action')
            if action=='start':
                if self.phase not in ('ready','complete') or type(message.get('expected_round')) is not int or message['expected_round']!=self.round:raise ValueError('本轮已开始或轮次过期')
                validate_parameters(message.get('mode'),message.get('forward_us'),message.get('duration_s'))
                self.mode=message['mode'];self.forward_us=message['forward_us'];self.duration_s=message['duration_s'];self.round+=1
                self.ever_forward=False;self.drive_started_s=self.brake_started_s=None;self.brake_count=0
                self._phase('arming',now,3.,1500,1610)
            elif action!='keepalive':raise ValueError('unsupported short-test command')
            self.sequence=seq;self.lease_until=now+.2
        except Exception as exc:
            self.abort(str(exc),now);raise

    def snapshot(self,now):
        return {'phase':self.phase,'reason':self.reason,'round':self.round,'mode':self.mode,
            'esc_us':self.esc_us,'steering_us':self.steering_us,'forward_us':self.forward_us,'duration_s':self.duration_s,
            'remaining_s':max(0,self.deadline-now) if self.deadline else 0.,'brake_count':self.brake_count,
            'drive_started_s':self.drive_started_s,'brake_started_s':self.brake_started_s,'source_time_s':now,
            'fault':self.fault,'physical_stop_verified':False}


def _worker(pipe,port):
    adapter=ownership=guard=None
    try:
        import fcntl
        from .rasadapter5 import RasAdapter
        ownership=open('/tmp/carvision-pi-pwm.lock','a+');fcntl.flock(ownership.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        adapter=RasAdapter(port);adapter.open();adapter.receive(.02)
        guard=ShortGuard(lambda ch,p:write_parking_pulse(adapter,ch,p,forward_limit_us=1625),time.monotonic())
        time.sleep(.03);readback=verify_neutral_readback(adapter)
        pipe.send({'ready':True,'readback':readback,'status':guard.snapshot(time.monotonic())})
        last_phase=None;next_report=0.
        while True:
            now=time.monotonic();guard.tick(now)
            if pipe.poll(.005):
                try:message=pipe.recv()
                except EOFError:guard.abort('parent ended',time.monotonic());break
                if message.get('close'):guard.abort('short-test session closed',time.monotonic());break
                try:guard.accept(message,time.monotonic())
                except ValueError:pass  # guard has already entered failsafe
            if guard.phase!=last_phase or now>=next_report:
                pipe.send({'status':guard.snapshot(time.monotonic())});last_phase=guard.phase;next_report=now+.1
            if guard.fault and guard.emergency_start is None and guard.emergency_until is None:break
    except BaseException as exc:
        try:pipe.send({'fault':str(exc)})
        except (OSError,EOFError):pass
    finally:
        if guard:
            try:
                guard.abort('worker ending',time.monotonic());end=time.monotonic()+.4
                while (guard.emergency_start is not None or guard.emergency_until is not None) and time.monotonic()<end:
                    guard.tick(time.monotonic());time.sleep(.005)
                guard._output(1500,1610)
            except Exception:pass
        if adapter:adapter.close()
        if ownership:ownership.close()
        pipe.close()


class ShortActuator:
    def __init__(self,*,run=False,esc_mode=None,port='/dev/ttyAMA0'):
        self.run=run;self.closed=False;self.sequence=0;self.process=self.pipe=None;self.latest={'phase':'ready','round':0}
        if run:
            if sys.platform!='linux' or esc_mode!='F/B':raise ValueError('short tests require this Pi and declared F/B mode')
            ctx=mp.get_context('spawn');self.pipe,child=ctx.Pipe()
            self.process=ctx.Process(target=_worker,args=(child,port),daemon=True);self.process.start();child.close()
            if not self.pipe.poll(3):self.close();raise ValueError('短测执行器启动超时')
            result=self.pipe.recv()
            if not result.get('ready'):self.close();raise ValueError('短测执行器未就绪：'+str(result))
            self.latest=result['status']

    def poll(self):
        if self.run:
            while self.pipe.poll():
                try:r=self.pipe.recv()
                except EOFError:break
                if r.get('fault'):raise ValueError(r['fault'])
                if r.get('status'):self.latest=r['status']
            if self.latest.get('fault'):raise ValueError(self.latest['fault'])
            if not self.process.is_alive():raise ValueError('短测输出进程已退出')
        return dict(self.latest)

    def send(self,action,**data):
        if self.closed:raise ValueError('short actuator closed')
        message={**data,'action':action,'sequence':self.sequence,'sent_s':time.monotonic()};self.sequence+=1
        if self.run:self.pipe.send(message)

    def close(self):
        if self.closed:return
        self.closed=True
        if self.pipe:
            try:self.pipe.send({'close':True})
            except (OSError,EOFError):pass
        if self.process:
            self.process.join(timeout=1)
            if self.process.is_alive():self.process.terminate();self.process.join(timeout=1)
        if self.pipe:self.pipe.close()

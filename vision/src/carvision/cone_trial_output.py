"""Cone selection inside the shared, user-supplied S3 steering envelope.

Validate actual S3 commands separately within the selected subset of the live
team-supplied 1350..1870 range. Reuse PulseGuard for ESC, lease and finite F/B timing;
its neutral steering placeholder is translated only after this S3 validation.
All logged/input commands retain the actual pulse, not the placeholder.
"""

from .steering import LEFT_US, RIGHT_US
import multiprocessing as mp
import sys
import time

from .brake_output import BrakeActuator,PulseGuard,write_parking_pulse,verify_neutral_readback


def validate_limits(right,left):
    if (type(right) is not int or type(left) is not int
            or not RIGHT_US<=right<=1570 or not 1650<=left<=LEFT_US):
        raise ValueError('锥桶转向仅允许右 1350–1570、左 1650–1870 μs')


class ConePulseGuard(PulseGuard):
    def __init__(self,settings,write,*,mode,right_us=RIGHT_US,left_us=LEFT_US):
        validate_limits(right_us,left_us)
        if not 1500<settings.get('pwm',{}).get('search_us',0)<=1600:
            raise ValueError('锥桶测试油门上限为 1600 μs')
        self.right_us,self.left_us=right_us,left_us;self.actual_steering=1610;self.actual_write=write
        super().__init__(settings,self._write,mode=mode,forward_limit_us=1625)

    def _write(self,channel,pulse):
        self.actual_write(channel,self.actual_steering if channel==3 else pulse)

    def neutral(self):
        self.actual_steering=1610
        super().neutral()

    def accept(self,command,now):
        self.tick(now)
        if self.fault:raise RuntimeError(self.fault)
        try:
            if not isinstance(command,dict):raise ValueError('command must be an object')
            steer=command.get('steering_us')
            if type(steer) is not int or not self.right_us<=steer<=self.left_us:
                raise ValueError('steering exceeds selected cone envelope')
            if command.get('action')=='neutral' and steer!=1610:raise ValueError('neutral must center S3')
            self.actual_steering=steer
            super().accept({**command,'steering_us':1610},now)
        except Exception as exc:
            self.abort(str(exc),now);raise


def _worker(connection,settings,port,mode,right_us,left_us):
    adapter=ownership=guard=None
    try:
        import fcntl
        from .rasadapter5 import RasAdapter
        ownership=open('/tmp/carvision-pi-pwm.lock','a+')
        fcntl.flock(ownership.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        adapter=RasAdapter(port);adapter.open();adapter.receive(.02)
        def write(channel,pulse):
            write_parking_pulse(adapter,channel,pulse,forward_limit_us=1625)
        guard=ConePulseGuard(settings,write,mode=mode,right_us=right_us,left_us=left_us)
        guard.neutral();time.sleep(.03)
        connection.send({'ready':True,'startup_readback':verify_neutral_readback(adapter)})
        reported=False
        while True:
            guard.tick(time.monotonic())
            if guard.fault and not reported:connection.send({'fault':guard.fault});reported=True
            if connection.poll(.005):
                try:message=connection.recv()
                except EOFError:
                    guard.abort('parent ended',time.monotonic());break
                if message.get('close'):
                    guard.abort('closed by parent',time.monotonic());break
                try:guard.accept(message,time.monotonic())
                except Exception:
                    if not reported:connection.send({'fault':guard.fault});reported=True
            if guard.fault and guard.emergency_until is None:break
    except BaseException as exc:
        try:connection.send({'fault':str(exc)})
        except (OSError,EOFError):pass
    finally:
        if guard:
            try:
                guard.abort('worker ending',time.monotonic());end=time.monotonic()+.35
                while guard.emergency_until is not None and time.monotonic()<end:
                    guard.tick(time.monotonic());time.sleep(.005)
                guard.neutral()
            except Exception:pass
        if adapter:adapter.close()
        if ownership:ownership.close()
        connection.close()


class ConeTrialActuator(BrakeActuator):
    def __init__(self,settings,*,right_us=RIGHT_US,left_us=LEFT_US,run=False,esc_mode=None,port='/dev/ttyAMA0'):
        validate_limits(right_us,left_us)
        super().__init__(settings,run=False,forward_limit_us=1625)
        self.run=run;self.steering_limits_us=[right_us,left_us]
        if not run:return
        if sys.platform!='linux' or esc_mode!='F/B':raise ValueError('live cone output requires Linux and F/B')
        ctx=mp.get_context('spawn');parent,child=ctx.Pipe();self.connection=parent
        self.process=ctx.Process(target=_worker,args=(child,settings,port,esc_mode,right_us,left_us),daemon=True)
        self.process.start();child.close()
        if not parent.poll(3):self.close();raise RuntimeError('cone actuator startup timed out')
        ready=parent.recv()
        if not ready.get('ready'):self.close();raise RuntimeError('cone UART/output unavailable: '+str(ready))
        self.startup_readback=ready.get('startup_readback')

    def send(self,intent):
        if type(intent.get('steering_us')) is not int or not self.steering_limits_us[0]<=intent['steering_us']<=self.steering_limits_us[1]:
            self.close();raise ValueError('steering exceeds selected cone envelope')
        return {**super().send(intent),'steering_limits_us':self.steering_limits_us}

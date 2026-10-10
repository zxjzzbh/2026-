"""Dedicated experimental F/B actuator; independent process, finite pulses.

Does not weaken the existing manual driver's allowed pulse values. The live
caller must explicitly declare F/B; the board cannot read that physical switch.
The default 1575-us parking envelope is unchanged. Only the traffic caller
explicitly selects the separate 1625-us tuning ceiling.
"""
import multiprocessing as mp
import struct
import sys
import time

from .brake_parking import numeric


def write_parking_pulse(adapter, channel, pulse, *, forward_limit_us=1575):
    """Reuse the original manual transport for the known forward/neutral points."""
    if type(forward_limit_us) is not int or forward_limit_us not in (1575, 1625):
        raise ValueError('unsupported forward output envelope')
    if type(pulse) is not int or channel not in (3, 4):
        raise ValueError('only parking S3/S4 output is allowed')
    if channel == 3:
        adapter.set_position(3, pulse)
    elif pulse in (1500, 1575):
        adapter.set_esc(4, pulse)
    elif 1300 <= pulse <= forward_limit_us:
        adapter.send(4, struct.pack('<BHBBH', 1, 20, 1, 4, pulse))
    else:
        raise ValueError('ESC pulse outside parking profile')


def verify_neutral_readback(adapter):
    """Stored board commands only; this does not prove motor/ESC readiness."""
    actual = {channel: adapter.read_position(channel) for channel in (3, 4)}
    if actual != {3: 1610, 4: 1500}:
        raise RuntimeError('S3/S4 neutral stored-command readback did not match')
    return {'steering_stored_us': actual[3], 'esc_stored_us': actual[4],
            'physical_stop_verified': False, 'esc_ready_verified': False}


class PulseGuard:
    def __init__(self,settings,write,*,mode,forward_limit_us=1575):
        if mode!='F/B':raise ValueError('active output requires the non-reversing F/B mode')
        if type(forward_limit_us) is not int or forward_limit_us not in (1575, 1625):
            raise ValueError('unsupported forward output envelope')
        p=settings.get('pwm',{})
        if (p.get('neutral_us')!=1500 or p.get('steering_center_us')!=1610
                or any(type(p.get(k)) is not int for k in ('search_us','creep_us','brake_us'))
                or not 1500<p['creep_us']<=p['search_us']<=forward_limit_us or not 1300<=p['brake_us']<1500
                or not numeric(settings.get('brake_pulse_s')) or not 0<settings['brake_pulse_s']<=.3
                or not numeric(settings.get('actuator_lease_s')) or not 0<settings['actuator_lease_s']<=.25
                or not numeric(settings.get('brake_release_s')) or settings['brake_release_s']<=0):
            raise ValueError('invalid local F/B actuator envelope')
        self.s,self.write=settings,write
        self.sequence=-1
        self.deadline=self.brake_until=self.emergency_until=None
        self.previous_action='neutral'
        self.esc=1500
        self.fault=None
        self.last_t=None
        self.last_brake_end=None

    def neutral(self):
        self.write(4,1500);self.esc=1500
        self.write(3,self.s['pwm']['steering_center_us'])

    def abort(self,reason,now):
        if self.fault:return
        self.fault=reason;self.deadline=None
        if self.esc>1500:
            self.write(4,self.s['pwm']['brake_us']);self.esc=self.s['pwm']['brake_us']
            self.emergency_until=now+self.s['brake_pulse_s']
        elif self.esc<1500 and self.brake_until is not None and now<self.brake_until:
            self.emergency_until=self.brake_until
        else:self.neutral()

    def tick(self,now):
        if not numeric(now):raise ValueError('invalid actuator clock')
        if self.last_t is not None and now<self.last_t:
            self.abort('clock reversed',self.last_t);return
        self.last_t=now
        if self.emergency_until is not None and now>=self.emergency_until:
            self.emergency_until=None;self.neutral()
        if self.brake_until is not None and now>=self.brake_until:
            self.last_brake_end=self.brake_until
            self.brake_until=None;self.write(4,1500);self.esc=1500
        if not self.fault and self.deadline is not None and now>=self.deadline:
            self.abort('command lease expired',now)

    def accept(self,command,now):
        self.tick(now)
        if self.fault:raise RuntimeError(self.fault)
        try:
            if not isinstance(command,dict):raise ValueError('command must be an object')
            seq,t=command.get('sequence'),command.get('sent_s')
            if type(seq) is not int or seq<=self.sequence or not numeric(t) or not 0<=now-t<=self.s['actuator_lease_s']:
                raise ValueError('stale, duplicate or future command')
            action=command.get('action');pulse=command.get('esc_us');steer=command.get('steering_us')
            expected={'neutral':1500,'search':self.s['pwm']['search_us'],
                      'creep':self.s['pwm']['creep_us'],'brake':self.s['pwm']['brake_us']}
            if action not in expected or type(pulse) is not int or pulse!=expected[action]:
                raise ValueError('action and pulse do not match the F/B profile')
            if type(steer) is not int or not 1570<=steer<=1650:raise ValueError('steering exceeds parking envelope')
            self.sequence=seq;self.deadline=now+self.s['actuator_lease_s']
            if action=='brake':
                if self.previous_action!='brake':
                    if self.last_brake_end is not None and now-self.last_brake_end<self.s['brake_release_s']:
                        raise ValueError('brake release interval not complete')
                    self.brake_until=now+self.s['brake_pulse_s']
                # Repeated frames cannot lengthen a single braking pulse.
                if self.brake_until is not None and now<self.brake_until:
                    self.write(4,pulse);self.esc=pulse
                else:self.write(4,1500);self.esc=1500
                self.write(3,steer)
            else:
                if self.esc<1500:self.last_brake_end=now
                self.brake_until=None
                if action=='neutral':self.neutral()
                else:
                    self.write(3,steer);self.write(4,pulse);self.esc=pulse
            self.previous_action=action
        except Exception as exc:
            self.abort(str(exc),now)
            raise


def _worker(connection,settings,port,mode,forward_limit_us=1575):
    adapter=ownership=guard=None
    try:
        import fcntl
        from .rasadapter5 import RasAdapter
        ownership=open('/tmp/carvision-pi-pwm.lock','a+')
        fcntl.flock(ownership.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        adapter=RasAdapter(port);adapter.open()
        adapter.receive(.02)
        def write(channel,pulse):
            # Local profile validation is narrower than the transport. Never
            # touch S1/S2; existing manual set_esc restrictions remain unchanged.
            write_parking_pulse(adapter,channel,pulse,forward_limit_us=forward_limit_us)
        guard=PulseGuard(settings,write,mode=mode,forward_limit_us=forward_limit_us)
        guard.neutral()
        time.sleep(.03)
        connection.send({'ready':True,'startup_readback':verify_neutral_readback(adapter)})
        reported=False
        while True:
            now=time.monotonic();guard.tick(now)
            if guard.fault and not reported:
                connection.send({'fault':guard.fault});reported=True
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
                guard.abort('worker ending',time.monotonic())
                end=time.monotonic()+.35
                while guard.emergency_until is not None and time.monotonic()<end:
                    guard.tick(time.monotonic());time.sleep(.005)
                guard.neutral()
            except Exception:pass
        if adapter:adapter.close()
        if ownership:ownership.close()
        connection.close()


class BrakeActuator:
    def __init__(self,settings,*,run=False,esc_mode=None,port='/dev/ttyAMA0',forward_limit_us=1575):
        if type(forward_limit_us) is not int or forward_limit_us not in (1575, 1625):
            raise ValueError('unsupported forward output envelope')
        self.run,self.s=run,settings
        self.sequence=0;self.connection=self.process=None
        self.startup_readback=None
        self.closed=False
        if run:
            if sys.platform!='linux' or esc_mode!='F/B':
                raise ValueError('live output requires Linux on the Pi and explicit --esc-mode F/B')
            ctx=mp.get_context('spawn');parent,child=ctx.Pipe()
            self.connection=parent
            self.process=ctx.Process(target=_worker,args=(child,settings,port,esc_mode,forward_limit_us),daemon=True)
            self.process.start();child.close()
            if not parent.poll(3):
                self.close();raise RuntimeError('actuator startup timed out')
            ready=parent.recv()
            if not ready.get('ready'):
                self.close();raise RuntimeError('UART ownership/output unavailable: '+str(ready))
            self.startup_readback=ready.get('startup_readback')

    def send(self,intent):
        if self.closed:raise RuntimeError('actuator closed')
        command={'sequence':self.sequence,'sent_s':time.monotonic(),'action':intent['action'],
                 'esc_us':intent['esc_us'],'steering_us':intent['steering_us']}
        self.sequence+=1
        if self.run:
            if self.connection.poll():raise RuntimeError(str(self.connection.recv()))
            if not self.process.is_alive():raise RuntimeError('actuator process ended')
            self.connection.send(command)
        return {'hardware_output':self.run,'command':command,'startup_readback':self.startup_readback,'tuning_verified':False}

    def close(self):
        if self.closed:return
        self.closed=True
        if self.connection:
            try:self.connection.send({'close':True})
            except (OSError,EOFError):pass
        if self.process:
            self.process.join(timeout=1)
            if self.process.is_alive():
                self.process.terminate();self.process.join(timeout=1)
        if self.connection:self.connection.close()

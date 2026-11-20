"""One whole sentence to the car's verified expansion-board I2C TTS.

Protocol retained from the team's existing autonomy_audio.tts_frames and
speak_parking.py. No UART, motor, servo or camera interfaces. A successful
exit means the command was delivered, never automatic proof of audible end.
The C++ caller enforces the child-process deadline and single-send policy.
"""
import argparse
import json
import os


def packet(text, volume=8):
    if not isinstance(text,str) or not text.strip() or type(volume) is not int or not 0<=volume<=10:
        raise ValueError('nonempty text and volume 0..10 required')
    content=(f'[h0][v{volume}]'+text).encode('gb2312')
    size=len(content)+2
    frame=bytes([0,0xFD,size>>8,size&255,1,0])+content
    if len(frame)>64:raise ValueError('whole sentence exceeds verified 64-byte application bound')
    return frame


class Transport:
    def __init__(self):self.fd=None
    def open(self):
        import fcntl
        self.fd=os.open('/dev/i2c-1',os.O_RDWR)
        try:
            fcntl.flock(self.fd,fcntl.LOCK_EX|fcntl.LOCK_NB)
            fcntl.ioctl(self.fd,0x0703,0x40)
        except BaseException:
            self.close();raise
    def status(self):
        value=os.read(self.fd,1)
        if len(value)!=1:raise OSError('incomplete TTS status')
        return value[0]
    def send(self,frame):return os.write(self.fd,frame)
    def close(self):
        if self.fd is not None:os.close(self.fd);self.fd=None


def send_once(text,volume=8,*,transport=None):
    frame=packet(text,volume)
    transport=transport or Transport()
    result=dict(backend='i2c_tts',bus=1,address=64,text=text,volume=volume,
                command_sent=False,completion_verified=False,motion_commands_sent=False)
    try:
        transport.open()
        # Status bytes observed on this car (0x4A/0x41) have no verified
        # busy->idle contract. Record them without guessing completion.
        try:result['status_before']=transport.status()
        except OSError as e:result['status_warning']=str(e)
        written=transport.send(frame)
        result['bytes_written']=written
        if written!=len(frame):raise OSError('incomplete TTS sentence write; do not split or resume')
        result['command_sent']=True
        # Do not wait for an unverified status after delivery: a blocked read
        # could hide a successful send and cause duplicate announcements.
        return result
    finally:transport.close()


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--text',required=True)
    parser.add_argument('--volume',type=int,default=8,choices=range(11))
    args=parser.parse_args()
    try:
        result=send_once(args.text,args.volume)
        print(json.dumps(result,ensure_ascii=False),flush=True)
    except Exception as error:
        print(json.dumps(dict(command_sent=False,completion_verified=False,error=str(error)),ensure_ascii=False),flush=True)
        raise SystemExit(2)

import sys
import unittest
from pathlib import Path

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'tools'))
from announce_i2c import packet,send_once


class Fake:
    def __init__(self,partial=False):self.frames=[];self.closed=False;self.partial=partial
    def open(self):pass
    def status(self):return 0x4A
    def send(self,frame):self.frames.append(frame);return len(frame)-int(self.partial)
    def close(self):self.closed=True


class TTSTransport(unittest.TestCase):
    def test_full_sentence_is_one_message_not_smbus_chunks(self):
        text='我是山东大学启航队的无人车，请为我加油！'
        frame=packet(text,8)
        self.assertGreater(len(frame),32)
        self.assertLessEqual(len(frame),64)
        self.assertEqual(frame[:2],b'\x00\xfd')
        self.assertEqual(int.from_bytes(frame[2:4],'big'),len(frame)-4)
        self.assertEqual(frame[6:].decode('gb2312'),'[h0][v8]'+text)
        t=Fake();r=send_once(text,8,transport=t)
        self.assertEqual(t.frames,[frame]);self.assertTrue(t.closed)
        self.assertTrue(r['command_sent']);self.assertFalse(r['completion_verified'])
        self.assertFalse(r['motion_commands_sent'])

    def test_partial_write_fails_and_is_not_retried(self):
        t=Fake(partial=True)
        with self.assertRaises(OSError):send_once('语音测试',8,transport=t)
        self.assertEqual(len(t.frames),1);self.assertTrue(t.closed)

    def test_bad_text_or_volume_cannot_open_hardware(self):
        for text,volume in [('',8),('测试',11),('加油'*30,8),('test🙂',8)]:
            t=Fake()
            with self.assertRaises((ValueError,UnicodeError)):send_once(text,volume,transport=t)
            self.assertEqual(t.frames,[])


if __name__=='__main__':unittest.main(verbosity=2)

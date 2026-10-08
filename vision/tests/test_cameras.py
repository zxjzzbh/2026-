import struct
import sys
import threading
from pathlib import Path
from types import SimpleNamespace

import pytest
import cv2
import numpy as np

from carvision.cameras import camera_selector, enumerate_linux_cameras, parse_capabilities
from carvision.sources import CameraSource
from carvision.web_preview import serve


def test_device_caps_override_aggregate_capture_flags_for_metadata_node():
    raw = struct.pack('=16s32s32s6I', b'uvcvideo', b'Test Camera', b'usb-test', 1,
                      0x80000000|0x00800000|1, 0x00800000, 0,0,0)
    info = parse_capabilities(raw)
    assert info['metadata_capture'] is True
    assert info['video_capture'] is False


def test_inventory_counts_physical_cameras_not_metadata_or_codec_nodes(tmp_path,monkeypatch):
    sysroot, devroot = tmp_path/'sys',tmp_path/'dev'
    sysroot.mkdir()
    devroot.mkdir()
    resolved = {}
    for n in range(4):
        parent = tmp_path/f'usb{n//2}'
        parent.mkdir(exist_ok=True)
        (parent/'idVendor').write_text('1234')
        (parent/'idProduct').write_text('5678')
        entry = sysroot/f'video{n}'
        entry.mkdir()
        (entry/'device').mkdir()
        resolved[entry/'device'] = parent
        (devroot/entry.name).touch()
    real_resolve = Path.resolve
    monkeypatch.setattr(Path,'resolve',lambda p,*a,**kw: resolved[p] if p in resolved else real_resolve(p,*a,**kw))
    def query(node):
        n=int(Path(node).name[5:])
        return {'driver':'uvcvideo','video_capture':n%2==0,'metadata_capture':n%2==1,'name':'camera'}
    result = enumerate_linux_cameras(sysroot,devroot,query)
    assert result['physical_usb_cameras']==2
    assert sum(d['selectable_video'] for d in result['nodes'])==2
    assert result['depth_stream_verified'] is False


@pytest.mark.parametrize('value',['video.mp4','http://camera/','-1',-1,True,.5,'/dev/v4l/by-id/../video0'])
def test_camera_selector_cannot_fall_back_to_files_or_urls(value):
    with pytest.raises(ValueError):
        camera_selector(value)


def test_stable_camera_identity_is_passed_without_index_fallback(monkeypatch):
    calls=[]
    cap=SimpleNamespace(isOpened=lambda:True,get=lambda p:30,set=lambda p,v:True,release=lambda:None)
    monkeypatch.setattr(sys,'platform','win32')
    monkeypatch.setattr('carvision.sources.cv2.VideoCapture',lambda *args: calls.append(args) or cap)
    monkeypatch.setattr(CameraSource,'_read',lambda self:None)
    path='/dev/v4l/by-id/usb-actual-camera-video-index0'
    source=CameraSource(path)
    source.close()
    assert calls==[(path,)]


def test_dual_preview_rejects_two_selectors_for_same_device_before_open():
    args=SimpleNamespace(port=8080,preview_fps=10,preview_width=640,jpeg_quality=75,
                         stale_ms=1500,aux_camera=0,camera=0)
    with pytest.raises(ValueError,match='same camera node'):
        serve(args,None)


def test_rejected_camera_format_stops_before_reader_and_does_not_fallback(monkeypatch):
    calls=[]
    cap=SimpleNamespace(isOpened=lambda:True,get=lambda p:cv2.VideoWriter_fourcc(*'YUYV'),
                        set=lambda p,v:False,release=lambda:calls.append('release'))
    monkeypatch.setattr(sys,'platform','win32')
    monkeypatch.setattr('carvision.sources.cv2.VideoCapture',lambda *args:cap)
    with pytest.raises(ValueError,match='no format fallback'):
        CameraSource(0,fourcc='MJPG')
    assert calls==['release']


def test_decode_limit_drains_skipped_frames_and_uses_the_latest_image(monkeypatch):
    decoded, released = [], []
    class Capture:
        index = 0
        def grab(self):
            self.index += 1
            return self.index <= 7
        def retrieve(self):
            decoded.append(self.index)
            return True, np.full((2, 2, 3), self.index, dtype=np.uint8)
        def release(self):
            released.append(True)
    source = object.__new__(CameraSource)
    source.cap = Capture()
    source.decode_interval = .1
    source._condition = threading.Condition()
    source._stop = threading.Event()
    source._latest = source._error = None
    times = iter([.01, .04, .08, .12, .15, .19, .24])
    monkeypatch.setattr('carvision.sources.time.monotonic', lambda: next(times))
    source._read()
    assert decoded == [1, 4, 7]
    assert np.all(source._latest.image == 7)
    assert source._latest.receive_monotonic_ms == 240
    assert isinstance(source._error, RuntimeError)
    assert released == [True]


def test_decode_limit_reports_camera_disconnect_without_reusing_a_frame():
    source = object.__new__(CameraSource)
    source.cap = SimpleNamespace(grab=lambda: False, release=lambda: None)
    source.decode_interval = .1
    source._condition = threading.Condition()
    source._stop = threading.Event()
    source._latest = source._error = None
    source._read()
    assert source._latest is None
    assert isinstance(source._error, RuntimeError)

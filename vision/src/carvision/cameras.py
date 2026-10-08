"""Read-only Linux V4L2 inventory; device nodes are not physical-camera counts."""

import os
import re
import stat
import struct
import sys
from pathlib import Path

QUERYCAP = 0x80685600
CAPTURE = 0x00000001 | 0x00001000
DEVICE_CAPS = 0x80000000
METADATA = 0x00800000


def camera_selector(value):
    """Only indexes or explicit Linux device paths, never a recording or URL."""
    if isinstance(value, int) and not isinstance(value, bool) and value >= 0:
        return value
    if isinstance(value, str):
        if value.isdecimal():
            return int(value)
        if re.fullmatch(r'/dev/video\d+', value) or value.startswith(('/dev/v4l/by-id/', '/dev/v4l/by-path/')):
            if '..' not in Path(value).parts:
                return value
    raise ValueError('camera must be a nonnegative index or /dev/videoN or /dev/v4l/by-id/by-path device')


def parse_capabilities(data):
    values = struct.unpack('=16s32s32s6I', data)
    decode = lambda b: b.split(b'\0', 1)[0].decode('utf-8', 'replace')
    caps = values[5] if values[4] & DEVICE_CAPS else values[4]
    return {'driver': decode(values[0]), 'name': decode(values[1]), 'bus': decode(values[2]),
            'video_capture': bool(caps & CAPTURE), 'metadata_capture': bool(caps & METADATA),
            'device_capabilities': caps}


def query_capabilities(path):
    import fcntl
    fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
    try:
        data = bytearray(104)
        fcntl.ioctl(fd, QUERYCAP, data, True)
        return parse_capabilities(data)
    finally:
        os.close(fd)


def device_path(selector):
    selector = camera_selector(selector)
    return Path(f'/dev/video{selector}' if isinstance(selector, int) else selector)


def validate_linux_camera(selector):
    path = device_path(selector)
    if not stat.S_ISCHR(path.stat().st_mode):
        raise ValueError(f'{path} is not a camera character device')
    info = query_capabilities(path)
    if not info['video_capture']:
        raise ValueError(f'{path} is not a video capture node (metadata/codec nodes cannot provide a camera image)')
    if info['driver'] != 'uvcvideo':
        raise ValueError(f'{path}: this source supports verified USB UVC video; other cameras need their own adapter')
    return info


def enumerate_linux_cameras(sys_root=Path('/sys/class/video4linux'), dev_root=Path('/dev'), query=None):
    query = query or query_capabilities
    aliases = {}
    for kind in ('by-id', 'by-path'):
        for link in sorted((dev_root/'v4l'/kind).glob('*')):
            aliases.setdefault(str(link.resolve()), []).append(str(link))
    devices = []
    for entry in sorted(sys_root.glob('video*'), key=lambda p: int(p.name[5:])):
        node = dev_root/entry.name
        row = {'node': str(node), 'aliases': aliases.get(str(node.resolve()), []),
               'sysfs_interface': str((entry/'device').resolve())}
        try:
            row.update(query(str(node)))
            parent = (entry/'device').resolve()
            usb = next((p for p in (parent, *parent.parents) if (p/'idVendor').is_file() and (p/'idProduct').is_file()), None)
            row['physical_device'] = str(usb) if usb else row['sysfs_interface']
            row['usb'] = None if usb is None else {name: (usb/name).read_text().strip() if (usb/name).exists() else None
                                                   for name in ('idVendor', 'idProduct', 'serial', 'product', 'manufacturer')}
            row['selectable_video'] = row['video_capture'] and row['driver'] == 'uvcvideo'
            row['depth_stream_verified'] = False
        except OSError as exc:
            row.update(error=str(exc), selectable_video=False)
        devices.append(row)
    physical = {d['physical_device'] for d in devices if d.get('selectable_video')}
    return {'platform': 'linux', 'physical_usb_cameras': len(physical), 'nodes': devices,
            'depth_stream_verified': False,
            'note': 'Video/metadata nodes can belong to the same camera. UVC video is not proof of metric depth.'}


def camera_inventory():
    if sys.platform != 'linux':
        return {'platform': sys.platform, 'nodes': [], 'physical_usb_cameras': None,
                'note': 'This inventory is for the Linux target; use the Windows USB probe on Windows.'}
    return enumerate_linux_cameras()

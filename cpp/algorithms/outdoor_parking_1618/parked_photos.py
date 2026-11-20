"""Capture a fresh still from each USB camera after motion and recording end."""
from pathlib import Path
import subprocess


CAMERAS = {
    'H65': 'usb-H65_USB_CAMERA_H65_USB_CAMERA-video-index0',
    'Jieli': 'usb-Jieli_Technology_USB_PHY_2.0-video-index0',
}


def capture_pair(folder):
    folder.mkdir(parents=True, exist_ok=True)
    results = {}
    for name, device in CAMERAS.items():
        # The Jieli stream has a green cast during its first frames after opening.
        warmup_frames = 60 if name == 'Jieli' else 5
        size = '1920x1080' if name == 'Jieli' else '1280x720'
        target = folder / ('parked_' + name + '.jpg')
        temporary = target.with_suffix('.tmp.jpg')
        try:
            subprocess.run([
                'ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-y',
                '-f', 'v4l2', '-input_format', 'mjpeg', '-video_size', size,
                '-i', str(Path('/dev/v4l/by-id') / device),
                '-vf', 'select=gte(n\\,%d)' % warmup_frames,
                '-frames:v', '1', '-q:v', '2', str(temporary),
            ], check=True, timeout=10, stdout=subprocess.DEVNULL,
               stderr=subprocess.PIPE)
            data = temporary.read_bytes()
            if not data.startswith(b'\xff\xd8') or not data.endswith(b'\xff\xd9'):
                raise ValueError('Incomplete JPEG')
            temporary.replace(target)
            results[name] = str(target)
        except (OSError, ValueError, subprocess.SubprocessError) as exc:
            detail = getattr(exc, 'stderr', None)
            results[name] = 'ERROR: ' + (detail.decode(errors='replace').strip() if detail else str(exc))
        finally:
            if temporary.exists():
                temporary.unlink()
    return results

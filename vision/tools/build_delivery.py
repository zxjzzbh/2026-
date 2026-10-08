"""Package source only, preserving a vision/ root and related interface docs."""

import argparse
import fnmatch
import hashlib
import json
from pathlib import Path
import zipfile


def build(output):
    source = Path(__file__).resolve().parents[1]
    repo = source.parent
    excluded = {'.venv', 'outputs', '__pycache__', '.pytest_cache', '.git'}
    ignored_extensions = {'.pyc', '.pyo', '.pt', '.pth', '.onnx', '.engine', '.tflite'}
    entries = []

    def collect(folder):
        for path in sorted(folder.iterdir()):
            if path.name in excluded or fnmatch.fnmatch(path.name.lower(), '*.egg-info') or path.is_symlink():
                continue
            if path.is_dir():
                collect(path)
            elif path.suffix.lower() not in ignored_extensions:
                entries.append(('vision/'+path.relative_to(source).as_posix(), path.read_bytes()))

    collect(source)
    interface = repo/'interfaces/vision-result-v0.1.md'
    if interface.is_file():
        entries.append(('interfaces/vision-result-v0.1.md', interface.read_bytes()))
    intro = '''# 视觉源码交接包（含浏览器实时预览）

请先阅读 vision/LIVE_PREVIEW.md，再阅读 vision/README.md。
本包只含源码、配置、测试和说明，不含系统镜像、Python 环境、录像或模型权重。
需要在树莓派重新建立适配的虚拟环境；不要套用 Windows CUDA 命令或 Windows 依赖锁文件。
当前源码要求 Python >=3.10，系统已烧录时先核对版本，不擅自重刷。
保留原有程序和配置，在独立目录解压并确认运行的是新版源码。

部署后，在树莓派 vision 目录运行（示例相机编号 0 需按实测修改）：
.venv/bin/python -m carvision serve --camera 0 --host 0.0.0.0 --port 8080
同一网络的电脑浏览器打开 http://树莓派实际IP:8080 。
页面显示真实相机原图/识别叠加/白线掩膜，不需要 VNC 或图形桌面。
启动时等待相机首帧，Ctrl+C 停止。端口只用于可信局域网调试。
当前没有比赛专用模型，YOLO disabled 正常；先验证真实采集和基础循迹。

已排除 .venv、outputs、__pycache__、*.egg-info，以及测试缓存和下载权重。
原文档引用的 outputs 历史产物不在本包内。
电脑测试通过不等于树莓派现场已验证；实测结果需要部署同学提供。
'''
    entries.append(('开始部署前请阅读.md', intro.encode('utf-8')))
    manifest = [{'path': name, 'bytes': len(data), 'sha256': hashlib.sha256(data).hexdigest()} for name, data in entries]
    entries.append(('MANIFEST.json', (json.dumps(manifest, ensure_ascii=False, indent=2)+'\n').encode('utf-8')))
    output = Path(output).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    with zipfile.ZipFile(output, 'x', compression=zipfile.ZIP_DEFLATED, compresslevel=9) as archive:
        for name, data in entries:
            archive.writestr(name, data)
    with zipfile.ZipFile(output) as archive:
        assert archive.testzip() is None
        for name, data in entries:
            assert archive.read(name) == data
            assert not any(p in excluded or p.endswith('.egg-info') for p in Path(name).parts)
        assert 'vision/src/carvision/web_preview.py' in archive.namelist()
    digest = hashlib.sha256(output.read_bytes()).hexdigest()
    output.with_suffix('.zip.sha256.txt').write_text(digest+'  '+output.name+'\n', encoding='utf-8')
    print(json.dumps({'archive': str(output), 'files': len(entries), 'bytes': output.stat().st_size,
                      'sha256': digest, 'verified': True}, ensure_ascii=True))


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    build(parser.parse_args().output)

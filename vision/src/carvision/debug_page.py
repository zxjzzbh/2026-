"""Standalone task-debug page using the same authenticated application server."""
from pathlib import Path
from types import SimpleNamespace

from .competition_page import add_static_page


def build_controls(*, root=None):
    folder = (Path(root)/'vision/web/debug' if root is not None
              else Path(__file__).resolve().parents[2]/'web/debug')
    return SimpleNamespace(HTML=(folder/'index.html').read_text(encoding='utf-8'),
                           SCRIPT=(folder/'app.js').read_text(encoding='utf-8'),
                           STYLE=(folder/'style.css').read_text(encoding='utf-8'))


def add_page(server, *, root=None, at_root=True):
    return add_static_page(server, build_controls(root=root), paths=('/debug', '/debug/'),
                           asset_prefix='/debug/assets', at_root=at_root)

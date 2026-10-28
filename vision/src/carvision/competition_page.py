"""Independent race page and static resources over the existing control API.

Only the generated server key/session is shared with the debug page. No manual
or debug HTML/client source is composed into the competition page.
"""
import json
import re
from pathlib import Path
from types import SimpleNamespace


def build_controls(base=None, *, root=None):
    """Compatibility entry for existing launchers; base is never mutated."""
    folder = (Path(root)/'vision/web/competition' if root is not None
              else Path(__file__).resolve().parents[2]/'web/competition')
    return SimpleNamespace(HTML=(folder/'index.html').read_text(encoding='utf-8'),
        SCRIPT=(folder/'app.js').read_text(encoding='utf-8'),
        STYLE=(folder/'style.css').read_text(encoding='utf-8'))


def render_page(original_html, controls=None):
    controls = controls or build_controls()
    match = re.search(r'const\s+key\s*=\s*("[^"\\]*")\s*,\s*pageSession\s*=\s*("[^"\\]*")', original_html)
    if not match:
        raise ValueError('original control server credentials unavailable')
    key, session = (json.loads(value) for value in match.groups())
    return _render_credentials(key, session, controls)


def _render_credentials(key, session, controls):
    if controls.HTML.count('__COMPETITION_BOOTSTRAP__') != 1:
        raise ValueError('competition bootstrap placeholder missing or repeated')
    bootstrap = json.dumps({'key': key, 'session': session}, ensure_ascii=True).replace('<', '\\u003c')
    return controls.HTML.replace('__COMPETITION_BOOTSTRAP__', bootstrap)


def add_page(server, controls=None, *, root=None, at_root=False, extra_paths=(), extra_path=None):
    """Register a race page and two explicit read-only static resources."""
    controls = controls or build_controls(root=root)
    aliases = tuple(extra_paths) + ((extra_path,) if extra_path else ())
    return add_static_page(server, controls, paths=('/competition', '/competition/', *aliases),
                           asset_prefix='/competition/assets', at_root=at_root, root_link=not aliases)


def add_static_page(server, controls, *, paths, asset_prefix, at_root=False, root_link=False):
    """Shared explicit resource registration for the race and debug services."""
    from urllib.parse import urlsplit
    assets = {asset_prefix+'/app.js': (controls.SCRIPT.encode('utf-8'), 'text/javascript; charset=utf-8'),
              asset_prefix+'/style.css': (controls.STYLE.encode('utf-8'), 'text/css; charset=utf-8')}
    original = server.RequestHandlerClass
    # New launchers expose these only in server memory. No legacy page or
    # JavaScript needs to be built or parsed on the independent race service.
    direct_page = None
    if getattr(server, 'drive_key', None) and getattr(server, 'drive_session', None):
        direct_page = _render_credentials(server.drive_key, server.drive_session, controls).encode('utf-8')

    class Handler(original):
        def do_GET(self):
            path = urlsplit(self.path).path
            if path in assets:
                content, mime = assets[path]
                return self.send_body(content, mime)
            if path not in paths and not (at_root and path == '/'):
                return super().do_GET()
            if direct_page is not None:
                return self.send_body(direct_page, 'text/html; charset=utf-8')
            before = self.path
            self.path = '/'
            self._competition_page = True
            try:
                return super().do_GET()
            finally:
                self.path = before
                self._competition_page = False

        def send_body(self, content, mime, status=200):
            if getattr(self, '_competition_page', False) and status == 200 and mime.startswith('text/html'):
                try:
                    content = render_page(content.decode('utf-8'), controls).encode('utf-8')
                except (ValueError, TypeError):
                    return super().send_body(b'Competition page bootstrap unavailable.', 'text/plain', 503)
            elif root_link and not at_root and status == 200 and mime.startswith('text/html') and urlsplit(self.path).path == '/':
                content = content.replace(b'</h1>', '</h1><p><a href="/competition">打开比赛操作页</a></p>'.encode('utf-8'), 1)
            return super().send_body(content, mime, status)

    server.RequestHandlerClass = Handler
    return server

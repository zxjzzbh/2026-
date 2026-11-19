"""5G HTTP control. GET is read-only; autonomous lifetime is vehicle-owned."""
import argparse
from http.cookies import SimpleCookie
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import signal
import threading
from urllib.parse import parse_qs, urlsplit

from .supervisor import Supervisor, public_session


class ControlServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self, address, supervisor, web):
        self.supervisor = supervisor
        self.web = Path(web)
        self.sessions = {}
        self.session_lock = threading.Lock()
        super().__init__(address, Handler)


class Handler(BaseHTTPRequestHandler):
    server_version = 'SmartCar5G/1.0'

    def log_message(self, format, *args):
        # Polling/preview is deliberately quiet; state transitions have task logs.
        pass

    def _send(self, code, data, content_type='application/json; charset=utf-8', cookie=None):
        if not isinstance(data, bytes):
            data = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(code)
        self.send_header('Content-Type', content_type)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        self.send_header('X-Content-Type-Options', 'nosniff')
        self.send_header('X-Frame-Options', 'DENY')
        self.send_header('Content-Security-Policy', "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: blob:; frame-ancestors 'none'")
        if cookie:
            self.send_header('Set-Cookie', cookie)
        self.end_headers()
        try:
            self.wfile.write(data)
        except (BrokenPipeError, ConnectionResetError):
            pass

    def _session(self, create=False):
        cookie = SimpleCookie()
        try:
            cookie.load(self.headers.get('Cookie', ''))
        except Exception:
            return None, None, None
        token = cookie.get('smartcar_session')
        token = token.value if token else None
        with self.server.session_lock:
            if token in self.server.sessions:
                return token, self.server.sessions[token], None
            if not create:
                return None, None, None
            if len(self.server.sessions) >= 128:
                raise PermissionError('会话数量过多，请重新启动页面服务后重试')
            token, csrf = secrets.token_urlsafe(32), secrets.token_urlsafe(32)
            self.server.sessions[token] = csrf
            return token, csrf, 'smartcar_session=' + token + '; Path=/; HttpOnly; SameSite=Strict'

    def _same_origin(self):
        if self.headers.get('Sec-Fetch-Site') == 'cross-site':
            raise PermissionError('拒绝跨站控制请求')
        origin = self.headers.get('Origin')
        if origin:
            parsed = urlsplit(origin)
            if parsed.scheme not in ('http', 'https') or parsed.netloc != self.headers.get('Host'):
                raise PermissionError('请求来源与控制页面不一致')

    def do_GET(self):
        route = urlsplit(self.path)
        try:
            if route.path == '/api/session':
                self._same_origin()
                session, csrf, cookie = self._session(create=True)
                return self._send(200, {'session': public_session(session), 'csrf': csrf}, cookie=cookie)
            if route.path == '/api/status':
                return self._send(200, self.server.supervisor.status())
            if route.path == '/api/frame.jpg':
                camera = parse_qs(route.query).get('camera', ['secondary'])[0]
                if camera not in ('primary', 'secondary'):
                    raise ValueError('未知摄像头')
                frame = self.server.supervisor.frame(camera)
                return self._send(200, frame, 'image/jpeg') if frame else self._send(503, {'error': '等待新画面'})
            static = {'/': ('index.html', 'text/html; charset=utf-8'),
                      '/competition': ('index.html', 'text/html; charset=utf-8'),
                      '/competition/': ('index.html', 'text/html; charset=utf-8'),
                      '/debug': ('index.html', 'text/html; charset=utf-8'),
                      '/debug/': ('index.html', 'text/html; charset=utf-8'),
                      '/static/app.js': ('app.js', 'application/javascript; charset=utf-8'),
                      '/static/style.css': ('style.css', 'text/css; charset=utf-8')}
            if route.path in static:
                name, content_type = static[route.path]
                return self._send(200, (self.server.web / name).read_bytes(), content_type)
            self._send(404, {'error': '页面不存在'})
        except PermissionError as exc:
            self._send(403, {'error': str(exc)})
        except (ValueError, OSError) as exc:
            self._send(400, {'error': str(exc)})

    def do_POST(self):
        try:
            if urlsplit(self.path).path != '/api/action':
                return self._send(404, {'error': '接口不存在'})
            self._same_origin()
            session, csrf, _ = self._session()
            if session is None:
                raise PermissionError('请刷新控制页面以建立会话')
            length = int(self.headers.get('Content-Length', '0'))
            if not 0 < length <= 8192:
                raise ValueError('请求长度无效')
            self.connection.settimeout(2)
            data = json.loads(self.rfile.read(length))
            if not isinstance(data, dict):
                raise ValueError('请求必须为对象')
            supplied = self.headers.get('X-CSRF-Token') or data.get('csrf')
            if not isinstance(supplied, str) or not secrets.compare_digest(csrf, supplied):
                raise PermissionError('控制会话校验失败，请刷新页面')
            self._send(200, self.server.supervisor.action(session, data))
        except PermissionError as exc:
            self._send(403, {'error': str(exc)})
        except (ValueError, OSError, RuntimeError) as exc:
            self._send(400, {'error': str(exc)})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[3])
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=8080)
    args = parser.parse_args()
    supervisor = Supervisor(args.root)
    try:
        server = ControlServer((args.host, args.port), supervisor, args.root / 'vision/web')
        def stop(*_):
            threading.Thread(target=server.shutdown, daemon=True).start()
        for name in ('SIGINT', 'SIGTERM'):
            signal.signal(getattr(signal, name), stop)
        try:
            print('5G interface ready; no UART opened and no vehicle start requested', flush=True)
            server.serve_forever(poll_interval=.1)
        finally:
            server.server_close()
    finally:
        supervisor.close()


if __name__ == '__main__':
    main()

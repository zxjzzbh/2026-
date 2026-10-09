"""Loopback-only capture UI; the car connection is strictly GET-only."""
import json
import secrets
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import urlsplit, parse_qs
from .traffic_preview import RemoteFeed,NoRedirect

from .traffic_capture import CAMERAS, TrafficCollector



def make_server(port,collector,asset):
    token=secrets.token_urlsafe(32)
    page=Path(asset).read_text(encoding='utf-8').replace('__CAPTURE_TOKEN__',json.dumps(token))

    class Handler(BaseHTTPRequestHandler):
        def log_message(self,*_):pass

        def send(self,data,mime='application/json; charset=utf-8',status=200):
            if not isinstance(data,bytes):data=json.dumps(data,ensure_ascii=False,allow_nan=False).encode()
            self.send_response(status)
            self.send_header('Content-Type',mime);self.send_header('Content-Length',str(len(data)))
            self.send_header('Cache-Control','no-store');self.send_header('X-Content-Type-Options','nosniff')
            self.end_headers();self.wfile.write(data)

        def allowed_host(self):
            return self.headers.get('Host') in (f'127.0.0.1:{self.server.server_port}',f'localhost:{self.server.server_port}')

        def do_GET(self):
            if not self.allowed_host():return self.send({'error':'loopback host required'},status=403)
            parsed=urlsplit(self.path);path=parsed.path;query=parse_qs(parsed.query)
            try:
                if path=='/':return self.send(page.encode(),'text/html; charset=utf-8')
                if path=='/api/status':return self.send(collector.status())
                if path=='/api/sessions':return self.send({'sessions':collector.sessions(),'active_session_id':collector.session_id})
                if path=='/api/samples':
                    return self.send(collector.samples(query.get('session',[None])[0],
                        int(query.get('page',['1'])[0]),int(query.get('page_size',['16'])[0]),
                        query.get('label',['all'])[0],query.get('camera',['all'])[0]))
                if path.startswith('/frame/') and path.endswith('.jpg'):
                    return self.send(collector.frame_bytes(path[len('/frame/'):-4]),'image/jpeg')
                if path.startswith('/sample/') and path.endswith('.jpg'):
                    sample_id=path[len('/sample/'):-4]
                    return self.send(collector.sample_bytes(query.get('session',[None])[0],sample_id),'image/jpeg')
                return self.send({'error':'not found'},status=404)
            except (ValueError,OSError) as exc:return self.send({'error':str(exc)},status=400)

        def do_POST(self):
            origin=self.headers.get('Origin')
            if (not self.allowed_host() or self.headers.get('X-Capture-Key')!=token
                    or origin and origin!='http://'+self.headers.get('Host','')):
                self.close_connection=True
                return self.send({'error':'采集页面已过期或来源无效，请刷新'},status=403)
            try:
                if self.headers.get('Transfer-Encoding') is not None or len(self.headers.get_all('Content-Length',[]))!=1:
                    raise ValueError('invalid request framing')
                length=int(self.headers.get('Content-Length','0'))
                if not 0<length<=8192 or self.headers.get_content_type()!='application/json':
                    raise ValueError('invalid JSON request')
                d=json.loads(self.rfile.read(length))
                if not isinstance(d,dict):raise ValueError('JSON object required')
                path=urlsplit(self.path).path
                if path=='/api/freeze':result={'token':collector.freeze(d.get('frames'))}
                elif path=='/api/capture':result=collector.capture(d.get('token'),d.get('camera'),d.get('label'),d.get('box'),d.get('metadata'))
                elif path=='/api/relabel':collector.relabel(d.get('sample_id'),d.get('label'),d.get('session_id'));result={'ok':True}
                elif path=='/api/session':result={'session_id':collector.new_session()}
                elif path=='/api/record':result={'sequence_id':collector.start_recording(d.get('cameras'),d.get('seconds',20),d.get('fps',3),d.get('metadata'))}
                elif path=='/api/record-stop':collector.stop_recording();result={'ok':True}
                elif path=='/api/export':result={'path':collector.export(d.get('session_id'))}
                elif path=='/api/exit':
                    self.send({'ok':True})
                    threading.Thread(target=self.server.shutdown,daemon=True).start();return
                else:return self.send({'error':'unsupported collection action'},status=404)
                self.send(result)
            except (ValueError,OSError,KeyError,TypeError) as exc:
                self.close_connection=True;self.send({'error':str(exc)},status=400)

    server=ThreadingHTTPServer(('127.0.0.1',port),Handler)
    server.daemon_threads=True
    return server


def run(preview,output,port,asset):
    collector=TrafficCollector(output)
    feed=RemoteFeed(preview,collector)
    server=make_server(port,collector,asset)
    try:
        feed.start()
        print(json.dumps({'app':'traffic-light-capture','url':f'http://127.0.0.1:{server.server_port}/',
                          'output':str(collector.root),'hardware_output':False},ensure_ascii=False),flush=True)
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:pass
    finally:
        feed.close();collector.stop_recording('program_closed');collector.export();server.server_close()

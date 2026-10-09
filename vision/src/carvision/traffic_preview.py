"""Read-only client for the already-running car camera preview service."""
import json
import threading
from urllib.parse import urlsplit
from urllib.request import build_opener, ProxyHandler, HTTPRedirectHandler, Request

CAMERAS = ('primary', 'secondary')


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self,*args,**kwargs): return None


class RemoteFeed:
    def __init__(self, preview, collector):
        parsed=urlsplit(preview)
        if (parsed.scheme!='http' or not parsed.hostname or parsed.username or parsed.password
                or parsed.path not in ('','/') or parsed.query or parsed.fragment):
            raise ValueError('preview must be an explicit HTTP origin without credentials')
        self.base=preview.rstrip('/')
        self.collector=collector
        self.stop=threading.Event()
        self.threads=[]

    def get(self,opener,path,limit):
        request=Request(self.base+path,method='GET',headers={'Cache-Control':'no-cache'})
        with opener.open(request,timeout=2) as response: data=response.read(limit+1)
        if len(data)>limit: raise ValueError('摄像头响应过大')
        return data

    def fetch_once(self,camera,opener):
        if camera not in CAMERAS:raise ValueError('unknown camera')
        status=json.loads(self.get(opener,'/api/status?camera='+camera,262144))
        raw=status.get('raw_preview') or {}
        age=raw.get('host_frame_age_ms')
        if raw.get('state')!='ok' or type(age) not in (int,float) or not 0<=age<=1000:
            raise ValueError('车端没有新鲜的原始预览画面')
        jpeg=self.get(opener,'/frame.jpg?camera='+camera+'&view=raw',4*1024*1024)
        # Existing preview API does not attach acquisition IDs to JPEGs. Keep
        # this status as a hint, never claim it identifies the exact JPEG.
        source={'preview_origin':self.base,'camera_identity':status.get('camera_identity'),
                'camera_capture':status.get('camera_capture'),'status_frame_id_hint':raw.get('frame_id'),
                'status_frame_age_ms':age,'status_is_exact_jpeg_timestamp':False,
                'transport':'HTTP raw-preview JPEG','native_resolution_claimed':False}
        return self.collector.publish(camera,jpeg,source)

    def _work(self,camera):
        opener=build_opener(ProxyHandler({}),NoRedirect())
        while not self.stop.is_set():
            try:self.fetch_once(camera,opener)
            except Exception as exc:self.collector.error(camera,str(exc))
            self.stop.wait(.2)

    def start(self):
        for camera in CAMERAS:
            t=threading.Thread(target=self._work,args=(camera,),name='collect-'+camera,daemon=True)
            self.threads.append(t);t.start()

    def close(self):
        self.stop.set()
        for t in self.threads:t.join(timeout=3)

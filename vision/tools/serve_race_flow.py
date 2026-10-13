"""Add full race mode above the original console; single-task panels remain.

All existing launcher options are forwarded unchanged. --race-live enables
the explicit UI handoff, never automatic startup or a remote driving command.
"""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'));sys.path.insert(0,str(ROOT/'vision/tools'))


def main():
    from carvision import web_preview,manual_controls
    from carvision.race_flow_console import RaceFlowDrive
    from carvision.race_flow_controls import extend_controls,add_endpoints
    import serve_parking_payment
    original=web_preview.make_server;argv=sys.argv[:];wrappers=[]
    live='--race-live' in sys.argv
    sys.argv=[x for x in sys.argv if x!='--race-live']
    def make_server(address,state,secondary=None,drive=None):
        if drive is None or secondary is None:raise ValueError('完整流程需要原控制服务及双摄缓存')
        extend_controls(manual_controls)
        full=RaceFlowDrive(drive,{'primary':state,'secondary':secondary},ROOT,live=live);wrappers.append(full)
        return add_endpoints(original(address,state,secondary,full),full)
    web_preview.make_server=make_server
    try:return serve_parking_payment.main()
    finally:
        for wrapper in wrappers:wrapper.close()
        web_preview.make_server=original;sys.argv=argv


if __name__=='__main__':main()

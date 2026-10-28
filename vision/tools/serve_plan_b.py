"""Add optional Plan B around the installed race/manual/payment launcher.

Forwards existing arguments; starts no driving, audio or payment on boot.
"""
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'));sys.path.insert(0,str(ROOT/'vision/tools'))


def main():
    from carvision import web_preview,manual_controls
    from carvision.plan_b_console import PlanBDrive
    from carvision.plan_b_controls import extend_controls
    import serve_race_flow
    original=web_preview.make_server;wrappers=[]
    def make_server(address,state,secondary=None,drive=None):
        if drive is None:raise ValueError('Plan B需要已有遥控服务')
        extend_controls(manual_controls)
        fallback=PlanBDrive(drive,ROOT);wrappers.append(fallback)
        return original(address,state,secondary,fallback)
    web_preview.make_server=make_server
    try:return serve_race_flow.main()
    finally:
        for wrapper in wrappers:wrapper.close()
        web_preview.make_server=original


if __name__=='__main__':main()

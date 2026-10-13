"""Add parking payment to the exact existing car console launcher.

All command-line arguments are forwarded unchanged. This keeps the installed
manual, crosswalk, traffic and cone stack; no duplicate camera/serial process.
"""
import importlib.util
import sys
from pathlib import Path

ROOT=Path(__file__).resolve().parents[2]
sys.path.insert(0,str(ROOT/'vision/src'))
sys.path.insert(0,str(ROOT/'vision/tools'))


def main():
    from carvision import web_preview,manual_controls
    from carvision.payment_console import PaymentConsoleDrive,attach_payment_capture
    from carvision.payment_controls import extend_controls,add_endpoints
    from carvision.departure_trial import DepartureTrialDrive
    from carvision.departure_controls import extend_controls as departure_controls,add_endpoints as departure_endpoints
    from carvision.parking_bay_console import ParkingBayDrive
    from carvision.parking_bay_controls import extend_controls as bay_controls,add_endpoints as bay_endpoints
    original=web_preview.make_server
    wrappers=[]
    def make_server(address,state,secondary=None,drive=None):
        if drive is None:raise ValueError('付款模块需要现有车端控制台')
        if secondary is not None:attach_payment_capture(secondary)
        extend_controls(manual_controls)
        departure_controls(manual_controls)
        bay_controls(manual_controls)
        states={'primary':state,**({'secondary':secondary} if secondary else {})}
        payment=PaymentConsoleDrive(drive,states,ROOT)
        departure=DepartureTrialDrive(payment,states,ROOT)
        bay=ParkingBayDrive(departure,states,ROOT)
        wrappers.extend([bay,departure,payment])
        return bay_endpoints(departure_endpoints(add_endpoints(original(address,state,secondary,bay),payment),departure),bay)
    web_preview.make_server=make_server
    path=ROOT/'vision/tools/serve_crosswalk_trial.py'
    spec=importlib.util.spec_from_file_location('existing_race_console',path)
    module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    try:return module.main(sys.argv[1:])
    finally:
        for wrapper in wrappers:wrapper.close()
        web_preview.make_server=original


if __name__=='__main__':main()

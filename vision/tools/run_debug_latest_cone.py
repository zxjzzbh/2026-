"""Original debug application plus one isolated latest-cone task. Author: 13246."""
from pathlib import Path
import sys

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'src'))


def main(argv=None):
    from carvision import application,debug_page
    from carvision.cone_latest_task import install
    from carvision.cone_latest_page import extend_controls
    original=debug_page.build_controls
    def build_controls(*,root=None):return extend_controls(original(root=root))
    install(application);debug_page.build_controls=build_controls
    return application.main('debug',argv)


if __name__=='__main__':main()

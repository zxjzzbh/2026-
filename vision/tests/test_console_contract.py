"""The compact UI must preserve the existing transport and ESC boundaries."""
import json
from html.parser import HTMLParser
from pathlib import Path
import pytest
from carvision.console_layout import build_page
from carvision.manual_controls import HTML, STYLE
from carvision.web_preview import PAGE
from carvision.pi5_pwm import validate_review


def test_dashboard_retains_unique_control_video_and_diagnostic_targets():
    class IDs(HTMLParser):
        def __init__(self):super().__init__();self.ids=[]
        def handle_starttag(self,tag,attrs):
            if dict(attrs).get('id'):self.ids.append(dict(attrs)['id'])
    parser=IDs();page=build_page(PAGE,HTML,STYLE);parser.feed(page)
    assert len(parser.ids)==len(set(parser.ids))
    assert set(('video','aux-video','camera','assist-enabled','assist-target',
                'drive-enable','drive-stop','drive-emergency','drive-reset','drive-finish',
                'drive-speed','gamepad-arm','test-mode','bench-ready','race','vision-details')) <= set(parser.ids)
    assert "let view='raw'" in page
    assert '按住 RB' not in page


def test_esc_only_workflow_keeps_neutral_boot_channel_and_stop_checks():
    root=Path(__file__).resolve().parents[1]
    review=json.loads((root/'deployment/boot-test-template.json').read_text())
    review.update(boot_id='current',esc_off_confirmed_by_user=True,neutral_physically_verified=True)
    assert review['operator_confirmation_scope']=='esc_state_only'
    assert review['wheels_on_ground_confirmed_by_user'] is False
    validate_review(review,'current',ground_continuous=True)
    for field in ('neutral_physically_verified','esc_off_confirmed_by_user',
                  'stop_physically_verified','disconnect_stop_physically_verified'):
        with pytest.raises(ValueError):validate_review({**review,field:False},'current',ground_continuous=True)
    with pytest.raises(ValueError):validate_review(review,'different-boot',ground_continuous=True)
    with pytest.raises(ValueError):validate_review({**review,'esc_channel':3},'current',ground_continuous=True)

"""Current boundary measurements must not retain fractional prediction drift."""
import numpy as np
import pytest

from carvision.parking_inner_edges import inner_line


@pytest.mark.parametrize('phase',[.1,.2,.201,.35])
def test_identical_observed_pixel_transition_has_identical_measured_position(phase):
    markings=np.zeros((120,220),np.uint8);markings[60:70]=255
    ground=np.zeros_like(markings);ground[70:]=255
    # All search bands hit the same tape ending at row 69 and floor starting
    # at row 70. Subpixel placement is a prediction, not new image evidence.
    line=inner_line(markings,ground,np.array([20.,70.+phase]),
                    np.array([200.,70.+phase]),np.array([110.,95.]))
    assert line is not None
    normal,intercept=line
    assert intercept/normal[1]==pytest.approx(70.,abs=1e-5)

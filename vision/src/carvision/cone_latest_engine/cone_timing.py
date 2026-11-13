"""One camera-result lifetime shared with the installed cone command sender."""
from ..cone_command import CONE_RESULT_MAX_AGE_S
from .brake_parking import numeric

FRAME_MAX_AGE_S=CONE_RESULT_MAX_AGE_S


def frame_fresh(captured,now):
    return numeric(captured) and numeric(now) and 0<=now-captured<FRAME_MAX_AGE_S


def frame_error(captured,now):
    if not numeric(captured):return '相机画面不可用，结束本轮'
    if captured>now:return '相机时间异常，结束本轮'
    return f'相机画面超过 {round(FRAME_MAX_AGE_S*1000)} ms 未更新（当前 {round((now-captured)*1000)} ms），结束本轮'

"""Latest original camera pixels for control; JPEG is a legacy input only.

Capture identity/time are never changed by resize, copying or preview work.
Readers own their image so a detector cannot modify another consumer's frame.
"""
import copy
import math

import cv2
import numpy as np


def control_frame(state, *, width=480):
    with state.condition:
        if state.state in ('error','stopped'):
            raise ValueError('camera unavailable')
        frame=getattr(state,'capture_frame',None)
        identity=copy.deepcopy(getattr(state,'identity',None))
        jpeg,fid,ms=state.raw_jpeg,state.raw_frame_id,state.raw_received_ms
        if frame is not None:
            image=frame.image
            fid,ms=frame.frame_id,frame.receive_monotonic_ms
            if (state.raw_frame_id,state.raw_received_ms)!=(fid,ms):jpeg=None
        else:image=None
    if type(fid) is not int or type(ms) not in (int,float) or not math.isfinite(ms):
        raise ValueError('camera has no valid captured frame')
    native=image is not None
    if image is None:
        if not jpeg:raise ValueError('camera has no image')
        image=cv2.imdecode(np.frombuffer(jpeg,np.uint8),cv2.IMREAD_COLOR)
    if image is None or image.ndim!=3 or image.shape[2]!=3:
        raise ValueError('camera image decode failed')
    height,actual_width=image.shape[:2]
    if native and width is not None and actual_width>width:
        image=cv2.resize(image,(width,max(1,round(height*width/actual_width))),interpolation=cv2.INTER_AREA)
    else:image=image.copy()
    return {'image':image,'frame_id':fid,'captured_ms':ms,'jpeg':jpeg,
            'identity':identity,'native':native}

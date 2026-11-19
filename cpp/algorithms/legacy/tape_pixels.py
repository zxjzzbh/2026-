"""Current-image tape evidence under local glare, shared by lane and parking.

Large achromatic highlights are unknown, not free ground or invented tape.
Only the segmentation changes: image coordinates and calibration stay intact.
"""
import cv2
import numpy as np

PROCESSING_REVISION='tape-local-contrast-20261110'


def shadow_white_mask(image):
    """Current narrow, low-saturation ridges on a darker colored floor.

    This mask is only a proposal: callers still check observed stripe widths,
    both floor flanks, curve geometry and original ground-color support.
    """
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    value=hsv[:,:,2]
    ridge=tape_contrast(cv2.GaussianBlur(value,(5,5),0))
    saturation=hsv[:,:,1]
    mask=((value>=85)&(saturation<=100)&(ridge>=12)).astype(np.uint8)*255
    mask[broad_glare(hsv)>0]=0
    mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((3,3),np.uint8))
    segments=cv2.HoughLinesP(mask,1,np.pi/180,30,
                            minLineLength=image.shape[0]*.14,maxLineGap=6)
    supported=np.zeros_like(mask)
    if segments is not None:
        for x1,y1,x2,y2 in segments[:,0]:
            if abs(y2-y1)>=image.shape[0]*.09 and abs(x2-x1)<=6*abs(y2-y1):
                cv2.line(supported,(x1,y1),(x2,y2),255,15)
    # Segments only select existing pixels; they never fill a missing stripe.
    mask=cv2.bitwise_and(mask,supported)
    return mask


def broad_glare(hsv):
    """Reject broad near-white highlights while retaining narrow tape strips."""
    bright=cv2.inRange(hsv,(0,0,235),(179,45,255))
    # This width exceeds the normal narrow-stripe proposals at 480 x 360.
    # Opening selects only a broad core; dilation removes its bright rim too.
    core=cv2.morphologyEx(bright,cv2.MORPH_OPEN,
                         cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(31,31)))
    return cv2.dilate(core,np.ones((5,5),np.uint8))


def white_tape_mask(hsv,minimum_value,maximum_saturation):
    mask=cv2.inRange(hsv,(0,0,minimum_value),(179,maximum_saturation,255))
    mask[broad_glare(hsv)>0]=0
    return mask


def tape_contrast(grey):
    """Local bright-stripe contrast at two widths, without changing color."""
    fine=cv2.morphologyEx(grey,cv2.MORPH_TOPHAT,np.ones((15,15),np.uint8))
    wide=cv2.morphologyEx(grey,cv2.MORPH_TOPHAT,np.ones((31,31),np.uint8))
    return cv2.max(fine,wide)


def _contrast_tape_image(image):
    """One bounded shadow recovery pass, only at existing bright thin details.

    Original hue/saturation and image coordinates are retained. Broad glare,
    flat floor and dark unknown pixels receive no boost; no holes are filled.
    """
    hsv=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    grey=cv2.cvtColor(image,cv2.COLOR_BGR2GRAY)
    contrast=tape_contrast(grey)
    supported=(hsv[:,:,1]<=70)&(hsv[:,:,2]>=100)&(contrast>=13)&(broad_glare(hsv)==0)
    boost=np.minimum(70,contrast.astype(np.int16)*2)
    value=hsv[:,:,2].astype(np.int16)
    hsv[:,:,2]=np.where(supported,np.minimum(255,value+boost),value).astype(np.uint8)
    return cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)


def local_tape_image(image):
    """Normalize local illumination once; retain original color and unknowns.

    CLAHE only supplies stripe contrast. The original glare mask, brightness
    floor and 70-level boost bound remain authoritative for the returned image.
    Callers must still validate ground/occupancy against the original frame.
    """
    original=cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
    normalized=original.copy()
    normalized[:,:,2]=cv2.createCLAHE(clipLimit=1.5,tileGridSize=(8,8)).apply(original[:,:,2])
    enhanced=cv2.cvtColor(_contrast_tape_image(cv2.cvtColor(normalized,cv2.COLOR_HSV2BGR)),cv2.COLOR_BGR2HSV)
    value=original[:,:,2].astype(np.int16)
    supported=(original[:,:,1]<=70)&(value>=100)&(broad_glare(original)==0)
    recovered=np.minimum(value+70,np.maximum(value,enhanced[:,:,2].astype(np.int16)))
    original[:,:,2]=np.where(supported,recovered,value).astype(np.uint8)
    converted=cv2.cvtColor(original,cv2.COLOR_HSV2BGR)
    result=image.copy();changed=supported&(recovered>value)
    result[changed]=converted[changed]
    return result

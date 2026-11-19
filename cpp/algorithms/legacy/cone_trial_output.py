"""Pure range check shared by the cone controller."""
from .steering import LEFT_US, RIGHT_US
def validate_limits(right,left):
    if (type(right) is not int or type(left) is not int
            or not RIGHT_US<=right<=1570 or not 1650<=left<=LEFT_US):
        raise ValueError('锥桶转向仅允许右 1350–1570、左 1650–1870 μs')

"""Numeric predicate from the legacy algorithm; no output driver."""
import math
def numeric(v):
    return type(v) in (int, float) and math.isfinite(v)

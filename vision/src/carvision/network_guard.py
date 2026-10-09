"""Fail closed unless another uplink is disabled or has no carrier."""
from pathlib import Path


def interface_is_inactive(name):
    interface = Path('/sys/class/net') / name
    # Wi-Fi firmware can retain carrier=1 even after radio/association stops.
    # IFF_UP=0 is positive evidence that the interface cannot carry traffic.
    flags = int((interface / 'flags').read_text().strip(), 16)
    if not flags & 1:
        return True
    return (interface / 'carrier').read_text().strip() == '0'

"""Compatibility shim for roomwatch_core -> scoutiq_core."""
from scoutiq_core import *
from scoutiq_core.ros_utils import *
from scoutiq_core.utils import *
def __getattr__(name):
    import scoutiq_core
    return getattr(scoutiq_core, name)


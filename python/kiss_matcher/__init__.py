# MIT License
#
# Copyright (c) 2025 Hyungtae Lim et al.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
"""
High-level Python façade for the KISS-Matcher C++ backend.
All public symbols from the compiled module are re-exported so users can
simply::
    import kiss_matcher as km
    cfg = km.KISSMatcherConfig(voxel_size=0.3)
"""
from importlib import import_module as _im
__version__ = "1.0.0"
__all__: list[str] = []
# Import the backend that CMake just built (“_kiss_matcher” lives inside
# the same package directory thanks to the change above).
try:
    # Preferred: wheel built with the extension placed in the package
    _backend = _im("kiss_matcher._kiss_matcher")
except ModuleNotFoundError:
    # Fallback: extension was installed at top level (site-packages/_kiss_matcher*.so)
    _backend = _im("_kiss_matcher")
# Re-export every non-private attribute so that they appear directly under
# the top-level `kiss_matcher` namespace.
for _name in dir(_backend):
    if not _name.startswith("_"):
        globals()[_name] = getattr(_backend, _name)
        __all__.append(_name)
# Keep a private reference so the module isn’t garbage-collected.
del _backend, _name, _im

# MIT License
#
# Copyright (c) 2022 Ignacio Vizzo, Tiziano Guadagnino, Benedikt Mersch, Cyrill
# Stachniss.
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
option(BUILD_SHARED_LIBS "TBB BUILD_SHARED_LIBS" ON)
option(TBBMALLOC_BUILD  "TBB TBBMALLOC_BUILD" ON)
option(TBB_EXAMPLES "TBB TBB_EXAMPLES" OFF)
option(TBB_STRICT "TBB TBB_STRICT" OFF)
option(TBB_TEST "TBB TBB_TEST" OFF)
#option(TBB_SANITIZE "TBB TBB_SANITIZE" "thread")

include(FetchContent)
FetchContent_Declare(TBB
    URL https://github.com/uxlfoundation/oneTBB/archive/refs/tags/v2022.0.0.tar.gz
    OVERRIDE_FIND_PACKAGE)
FetchContent_MakeAvailable(TBB)


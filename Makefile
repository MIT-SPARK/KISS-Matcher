# To account for docker env
# SUDO := $(shell if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then echo "sudo"; else echo ""; fi)
ifeq ($(shell test -f /.dockerenv && echo -n yes),yes)
    SUDO :=
else
    SUDO := sudo
endif

# Detect the host OS so we can pick platform-specific commands.
UNAME_S := $(shell uname -s 2>/dev/null || echo Unknown)

ifeq ($(UNAME_S),Darwin)
    # macOS uses Homebrew. AppleClang ships without OpenMP support, so we
    # explicitly use the Homebrew LLVM clang toolchain for the C++ build.
    PARALLEL    := $(shell sysctl -n hw.ncpu 2>/dev/null || echo 4)
    LLVM_PREFIX := $(shell brew --prefix llvm 2>/dev/null)
    ifneq ($(LLVM_PREFIX),)
        CMAKE_EXTRA_ARGS := -DCMAKE_C_COMPILER=$(LLVM_PREFIX)/bin/clang -DCMAKE_CXX_COMPILER=$(LLVM_PREFIX)/bin/clang++
    else
        CMAKE_EXTRA_ARGS :=
    endif
else
    PARALLEL         := $(shell nproc --all 2>/dev/null || echo 4)
    CMAKE_EXTRA_ARGS :=
endif

ifeq ($(UNAME_S),Darwin)
deps:
	@echo "[KISS-Matcher] Installing dependencies for macOS..."
	@command -v brew >/dev/null 2>&1 || { \
	    echo "[KISS-Matcher] ERROR: Homebrew is required on macOS (https://brew.sh)"; \
	    exit 1; }
	@brew install cmake ninja eigen tbb lz4 flann llvm libomp ccache
else
deps:
	@echo "[KISS-Matcher] SUDO is: $(SUDO)"
	@echo "[KISS-Matcher] Installing dependencies..."
	@$(SUDO) apt-get update -y
	@$(SUDO) apt-get install -y gcc g++ build-essential libeigen3-dev python3-pip python3-dev cmake git ninja-build libflann-dev
endif

# I used this one:
# https://patorjk.com/software/taag/#p=display&f=ANSI%20Shadow
ascii_art:
	@echo " "
	@echo "██╗  ██╗██╗███████╗███████╗      "
	@echo "██║ ██╔╝██║██╔════╝██╔════╝      "
	@echo "█████╔╝ ██║███████╗███████╗█████╗"
	@echo "██╔═██╗ ██║╚════██║╚════██║╚════╝"
	@echo "██║  ██╗██║███████║███████║      "
	@echo "╚═╝  ╚═╝╚═╝╚══════╝╚══════╝      "
	@echo " "
	@echo "███╗   ███╗ █████╗ ████████╗ ██████╗██╗  ██╗███████╗██████╗ "
	@echo "████╗ ████║██╔══██╗╚══██╔══╝██╔════╝██║  ██║██╔════╝██╔══██╗"
	@echo "██╔████╔██║███████║   ██║   ██║     ███████║█████╗  ██████╔╝"
	@echo "██║╚██╔╝██║██╔══██║   ██║   ██║     ██╔══██║██╔══╝  ██╔══██╗"
	@echo "██║ ╚═╝ ██║██║  ██║   ██║   ╚██████╗██║  ██║███████╗██║  ██║"
	@echo "╚═╝     ╚═╝╚═╝  ╚═╝   ╚═╝    ╚═════╝╚═╝  ╚═╝╚══════╝╚═╝  ╚═╝"
	@echo " "

# Also install MIT-SPARK ROBIN
# See https://github.com/MIT-SPARK/ROBIN
# ToDo(hlim): It's not elegant, but at least it works
# Force FetchContent for ROBIN so a stale system install is not picked up;
# the goal of `cppinstall` is to (re)install ROBIN cleanly.
cppinstall: deps
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_ROBIN=OFF $(CMAKE_EXTRA_ARGS)
	@cmake --build cpp/kiss_matcher/build -j$(PARALLEL)
	@$(SUDO) cmake --install cpp/kiss_matcher/build
	@$(SUDO) cmake --install cpp/kiss_matcher/build/_deps/robin-build

cppinstall_matcher_only:
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release $(CMAKE_EXTRA_ARGS)
	@cmake --build cpp/kiss_matcher/build -j$(PARALLEL)
	@$(SUDO) cmake --install cpp/kiss_matcher/build

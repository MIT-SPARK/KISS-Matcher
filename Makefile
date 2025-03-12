# To account for docker env
# SUDO := $(shell if command -v sudo >/dev/null 2>&1 && sudo -n true 2>/dev/null; then echo "sudo"; else echo ""; fi)
ifeq ($(shell test -f /.dockerenv && echo -n yes),yes)
    SUDO :=
else
    SUDO := "sudo"
endif

deps:
	@echo "[KISS-Matcher] SUDO is: $(SUDO)"
	@echo "[KISS-Matcher] Installing dependencies..."
	@$(SUDO) apt-get update -y
	@$(SUDO) apt-get install -y gcc g++ build-essential libeigen3-dev python3-pip python3-dev cmake git ninja-build libflann-dev

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
cppinstall: deps ascii_art
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release
	@cmake --build cpp/kiss_matcher/build -j$(nproc --all)
	@$(SUDO) cmake --install cpp/kiss_matcher/build
	@$(SUDO) cmake --install cpp/kiss_matcher/build/_deps/robin-build

cppinstall_matcher_only: ascii_art
	@mkdir -p cpp/kiss_matcher/build
	@cmake -Bcpp/kiss_matcher/build cpp/kiss_matcher -DCMAKE_BUILD_TYPE=Release
	@cmake --build cpp/kiss_matcher/build -j$(nproc --all)
	@$(SUDO) cmake --install cpp/kiss_matcher/build

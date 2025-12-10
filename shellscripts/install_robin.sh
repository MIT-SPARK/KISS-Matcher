#!/usr/bin/env bash
set -e

command_exists() {
  command -v "$@" >/dev/null 2>&1
}

user_can_sudo() {
  command_exists sudo || return 1
  ! LANG= sudo -n -v 2>&1 | grep -q "may not run sudo"
}

if user_can_sudo; then
SUDO="sudo"
else
SUDO="" # To support docker environment
fi

if [ "$1" == "assume-yes" ]; then
    APT_CONFIRM="--assume-yes"
else
    APT_CONFIRM=""
fi

$SUDO apt-get update -y

# Automatically install the prerequisites
# `libflann-dev` is required for KISS-Matcher
$SUDO apt-get install gcc g++ build-essential libeigen3-dev python3-pip python3-dev cmake git ninja-build libflann-dev -y

# Install ROBIN
echo "
-------------------------------------------------------------------------------
------====--------========-----------------------========-------===-------=====
--*##########----+##########*--------+=---------=###########---=###=-----####+-
+####*====+####--+##======+###+-----=+*+--------=##+=====++##-*=###=---+####---
###+--------=###-+##=-------+##+---==+**=-------=##=-------=##*=###=--*###+----
###+-------------+##=--------##*--====***=------=##=--------##-=###==####=-----
+############*=--+##=-------+##=-===---+++=-----=##=-------=##-=#######+-------
---=+***########-+##======*###+-===-----+++=----=##======+##*--=#########=-----
--=----------*##-+##=######*---===-------+++=---=##=*####*=----=####*-*###*----
###*--------+###-+##=---------===---------=++=--=##=-==+###----=###=---=####=--
-##############=-+##=--------================+=-=##=-----###+--=###=-----*###*-
--=+########*=---+##=-------=========+=====++++==##=------+###*=###=------=####
--------------------------------------------------------------------------------
"

mkdir -p install && cd install
git clone https://github.com/MIT-SPARK/ROBIN.git
mkdir -p ROBIN/build && cd ROBIN/build

# To generate `libpmc.a`
# See https://github.com/jingnanshi/pmc/pull/2
cmake -DPMC_BUILD_SHARED=OFF ..

if make -j$(nproc); then
    echo "Complete to Build ROBIN successfully!"
else
    echo "Build failed, try 'make' several times ..."
    exit 1
fi

echo "Applying 'sudo make install'. Enter password"
$SUDO make install

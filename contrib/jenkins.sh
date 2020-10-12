#!/bin/sh
# jenkins build helper script for libosmo-abis.  This is how we build on jenkins.osmocom.org

if ! [ -x "$(command -v osmo-build-dep.sh)" ]; then
	echo "Error: We need to have scripts/osmo-deps.sh from http://git.osmocom.org/osmo-ci/ in PATH !"
	exit 2
fi

set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

osmo-clean-workspace.sh

mkdir "$deps" || true

verify_value_string_arrays_are_terminated.py $(find . -name "*.[hc]")

osmo-build-dep.sh libosmocore "" --disable-doxygen

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"
export PATH="$inst/bin:$PATH"

osmo-build-dep.sh osmo-e1d

set +x
echo
echo
echo
echo " =============================== libosmo-abis ==============================="
echo
set -x

autoreconf --install --force
./configure --enable-sanitize --enable-werror --enable-e1d
$MAKE $PARALLEL_MAKE
$MAKE $PARALLEL_MAKE distcheck \
  || cat-testlogs.sh
$MAKE $PARALLEL_MAKE maintainer-clean

osmo-clean-workspace.sh

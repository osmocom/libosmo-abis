#!/usr/bin/env bash

set -ex

rm -rf deps/install
mkdir deps || true
cd deps
osmo-deps.sh libosmocore

cd libosmocore
autoreconf --install --force
./configure --prefix=$PWD/../install
$MAKE $PARALLEL_MAKE install

cd ../../
autoreconf --install --force
export PKG_CONFIG_PATH=$PWD/deps/install/lib/pkgconfig:$PKG_CONFIG_PATH
export LD_LIBRARY_PATH=$PWD/deps/install/lib
./configure
$MAKE $PARALLEL_MAKE
$MAKE distcheck

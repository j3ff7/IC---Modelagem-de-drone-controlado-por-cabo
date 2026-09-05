#!/usr/bin/env bash
set -euo pipefail

cd /home/lima/codes/ic/drone-cabo
mkdir -p build/gz_plugins

g++ -std=c++17 -fPIC -shared \
  src/pacote_do_drone/gz_plugins/TetherForceConstraint.cc \
  -o build/gz_plugins/libTetherForceConstraint.so \
  $(pkg-config --cflags --libs gz-sim7 gz-plugin2 gz-transport13 gz-msgs10 gz-math7)

echo "/home/lima/codes/ic/drone-cabo/build/gz_plugins/libTetherForceConstraint.so"

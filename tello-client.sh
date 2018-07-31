#!/usr/bin/env bash
# n.b requires pygame
# ports 9000, 9617, 6038 -- 6038 is used for video stream, not sure about others

PYTHON="${PYTHON:=python3}"
#mkdir -p ~/Pictures/
PYSITE="$("$PYTHON" -m site --user-site)"
mkdir -p "$PYSITE"
cd "$(dirname "$(readlink -f $0)")"
ln -sf "$(pwd)/tellopy" "$PYSITE/"
python3 tellopy/examples/keyboard_and_video.py

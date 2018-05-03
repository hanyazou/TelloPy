#!/bin/bash
dir=$(cd "$(dirname "$0")"; pwd)

source "$dir"/wait_wifi_connection.bash
export PYTHONPATH="$dir"/..:$PYTHONPATH

wait_wifi_connection
python "$dir"/test.py

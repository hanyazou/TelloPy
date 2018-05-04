#!/bin/bash
dir=$(cd "$(dirname "$0")"; pwd)

source "$dir"/wait_wifi_connection.bash
export PYTHONPATH="$dir"/..:$PYTHONPATH

while sleep 1; do
    wait_wifi_connection
done

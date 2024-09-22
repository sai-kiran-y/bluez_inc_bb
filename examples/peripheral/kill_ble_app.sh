#!/bin/bash
PIDS=$(pgrep -f "ble_app")
if [ -z "$PIDS" ]; then
        echo "No running ble_app process(es) found"
else
        kill -9 $PIDS
        echo "Killed ble_app processes
fi


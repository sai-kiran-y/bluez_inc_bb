#!/bin/bash

LOG_FILE="/home/root/ble_app.log"
MAX_SIZE=$((500 * 1024 * 1024))  # 500MB in bytes

if [ -f "$LOG_FILE" ]; then
    FILE_SIZE=$(stat -c%s "$LOG_FILE")
    if [ "$FILE_SIZE" -ge "$MAX_SIZE" ]; then
        echo "Log file size is greater than or equal to 500MB. Clearing contents."
        > "$LOG_FILE"
    else
        echo "Log file size is under 500MB."
    fi
else
    echo "Log file does not exist."
fi


# Chrontab commands
# crontab -e
# 0 * * * * /usr/local/bin/ble_app_log_check_size.sh


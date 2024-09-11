#!/bin/sh

# BLE Application details
BLE_APP_NAME="ble_app"

# Function to kill running instances of the application
kill_app_instances() {
    # Find the PIDs of the running instances by the application name
    PIDS=$(pgrep -f "$BLE_APP_NAME")
    if [ -n "$PIDS" ]; then
        echo "Stopping running instances of $BLE_APP_NAME with PIDs: $PIDS..."
        kill -9 $PIDS
        sleep 2  # Wait for processes to terminate
    else
        echo "No running instances of $BLE_APP_NAME found."
    fi
}

# Function to clean up zombie processes
cleanup_zombies() {
    echo "Checking for zombie processes..."
    ZOMBIES=$(ps -el | grep 'Z' | grep "$BLE_APP_NAME" | awk '{print $4}')
    if [ ! -z "$ZOMBIES" ]; then
        echo "Found zombie processes. Cleaning up..."
        for PPID in $ZOMBIES; do
            echo "Killing parent process $PPID"
            kill -9 $PPID
        done
    else
        echo "No zombie processes found."
    fi
}

# Kill existing instances and clean up zombies before starting a new one
kill_app_instances
cleanup_zombies

# Initialize Bluetooth hardware
killall -9 hciattach
rfkill unblock all
hciattach /dev/ttySTM2 any 115200 noflow
sleep 1
hciconfig hci0 up
hcitool -i hci0 cmd 0x03 0x03

# Start the BLE application
echo "Starting $BLE_APP_NAME..."
/home/root/$BLE_APP_NAME 2>&1 | ts '[%Y-%m-%d %H:%M:%S]' > /home/root/ble_app.log

echo "$BLE_APP_NAME started successfully."


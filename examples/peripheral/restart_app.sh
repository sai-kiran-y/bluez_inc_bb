
#!/bin/bash

# Application details
APP_NAME="ble_init.sh"  # Your application name
BLE_APP_NAME="ble_app"  # Actual BLE app name
PID_FILE="/home/root/my_app.pid"  # Path to PID file

# Function to kill running instances of the application
kill_app_instances() {
    # Find the PIDs of the running instances by the application name
    PIDS=$(pgrep -f "$BLE_APP_NAME|$APP_NAME|restart_app.sh")
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
    ZOMBIES=$(ps -el | grep 'Z' | grep "$BLE_APP_NAME|$APP_NAME|$restart_app.sh" | awk '{print $4}')
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

# Check if the PID file exists and is not empty
if [ -s "$PID_FILE" ]; then
    # Read the PID from the file
    PID=$(cat "$PID_FILE")

    # Check if a process with this PID is running
    if ps -p $PID > /dev/null 2>&1; then
        echo "Stopping the application with PID $PID..."
        kill -9 $PID  # Force kill the process
        sleep 2  # Wait for the process to terminate
    else
        echo "Process with PID $PID is not running."
    fi

    # Remove the PID file after stopping the process
    rm -f "$PID_FILE"
else
    echo "PID file does not exist or is empty."
    echo "Searching for running instances of $BLE_APP_NAME..."
    # Kill running instances if the PID file is missing
    kill_app_instances
fi

# Clean up any zombie processes before restarting the application
cleanup_zombies

# Ensure all instances of ble_app are killed before starting again
kill_app_instances

# Start the application again
echo "Starting the application..."
/home/root/$APP_NAME 2>&1 | ts '[%Y-%m-%d %H:%M:%S]'> /home/root/ble_logs.txt # Replace with the command to start your app

# Save the new PID to the PID file
NEW_PID=$!
echo $NEW_PID > "$PID_FILE"

echo "Application restarted successfully with PID $NEW_PID."


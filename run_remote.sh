#!/bin/bash

# Check if host parameter is provided
if [ $# -ne 1 ]; then
    echo "Error: Host parameter required (1 or 2)"
    echo "Usage: $0 <host_number>"
    exit 1
fi

# Validate host parameter
if [ "$1" != "1" ] && [ "$1" != "2" ]; then
    echo "Error: Host parameter must be 1 or 2"
    echo "Usage: $0 <host_number>"
    exit 1
fi

# SSH connection details
if [ "$1" = "1" ]; then
    HOST="192.168.237.51"
else
    HOST="192.168.237.52"
fi
USER="raspi"

# IP address of the Prosody server
PROSODY_IP="192.168.237.165"

# Local agent folder path
LOCAL_AGENT_PATH="./agent"
# Remote path where to copy the agent folder
REMOTE_PATH="/home/raspi/app"

# Copy changes into the remote server
rsync -av --progress --exclude 'calibration_agent/.venv' --exclude 'calibration_agent/__pycache__' ./* ${USER}@${HOST}:${REMOTE_PATH}

# Then connect and restart docker compose
echo "Starting the stack in attached mode..."
ssh -t ${USER}@${HOST} "export PROSODY_IP=${PROSODY_IP};cd /home/raspi/app && docker compose down && docker compose up alphabot_agent_$1"

echo "Docker-compose stopped.."

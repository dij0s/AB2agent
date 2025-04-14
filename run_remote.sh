#!/bin/bash

# SSH connection details
HOST="192.168.237.51"
USER="raspi"

# IP address of the Prosody server
PROSODY_IP="192.168.237.57"

# Local agent folder path
LOCAL_AGENT_PATH="./agent"
# Remote path where to copy the agent folder
REMOTE_PATH="/home/raspi/app"


# Copy changes into the remote server
rsync -av --progress ./* ${USER}@${HOST}:${REMOTE_PATH}

# Then connect and restart docker compose
echo "Starting the stack in attached mode..."
ssh -t ${USER}@${HOST} "export PROSODY_IP=${PROSODY_IP};cd /home/raspi/app && docker compose down && docker compose up alphabot_agent"

echo "Docker-compose stopped.."

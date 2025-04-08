#!/bin/bash

# SSH connection details
HOST="192.168.237.135"
USER="raspi"

# Local agent folder path
LOCAL_AGENT_PATH="./agent"
# Remote path where to copy the agent folder
REMOTE_PATH="/home/raspi/app"

# Copy changes into the remote server
rsync -av --progress ${LOCAL_AGENT_PATH} ${USER}@${HOST}:${REMOTE_PATH}

# Then connect and restart docker compose
echo "Starting the stack in attached mode..."
ssh -t ${USER}@${HOST} 'cd /home/raspi/app && docker compose down && docker compose up'

echo "Docker-compose stopped.."

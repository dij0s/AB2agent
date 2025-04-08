# AB2 Agent

This repository contains the AB2 agent service that needs to be deployed on a Raspberry Pi.
It listens for incoming XMPP messages and processes them accordingly.

## Prerequisites

1. SSH access to your Raspberry Pi configured with certificate authentication
2. Docker and Docker Compose installed on the Raspberry Pi
3. rsync installed on your local machine

## SSH Configuration

1. Make sure you have your SSH key pair ready
2. Configure your SSH by adding these lines to your `~/.ssh/config`:
```
Host 192.168.237.*
   HostName %h
   PreferredAuthentications publickey
   User raspi
   IdentityFile ~/.ssh/keys/your-public-key
```

## Deployment

1. Clone this repository:
```bash
git clone <repository-url>
cd AB2agent
```

2. Adjust the `HOST` variable in `run_remote.sh` if your Raspberry Pi has a different IP address

3. Run the deployment script:
```bash
./run_remote.sh
```

This will:
- Sync any changed files to the Raspberry Pi using rsync
- Start the Docker Compose stack in attached mode
- Show the logs in your terminal
- Allow you to stop the stack with CTRL+C

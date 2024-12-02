#!/bin/bash

#xhost +

# Get the directory of the current script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

source ./simulator.env
TIMEOUT=60 CONF_FILE=$CONF_FILE_PASS docker compose -f "$SCRIPT_DIR/docker-compose.yml" up web-visualizer -d
# Use the script directory to make the docker-compose.yml path relative

while true; do
    	echo "Updating planning container.."
        echo "Running planning v1.."
	TIMEOUT=70 CONF_FILE=$CONF_FILE_FAIL docker compose -f "$SCRIPT_DIR/docker-compose.yml" up planning-control simulator --abort-on-container-exit
        echo "Updating planning container.."
        echo "Running planning v2.."
	TIMEOUT=120 CONF_FILE=$CONF_FILE_PASS docker compose -f "$SCRIPT_DIR/docker-compose.yml" up planning-control simulator --abort-on-container-exit
done

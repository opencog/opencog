#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints the size of incoming message queue (messageCentral)"
    echo "Requires DEBUG log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

echo "Please be sure that log level is at DEBUG"
./tailgrep_from_start.sh "${1}" "messageCentral size"

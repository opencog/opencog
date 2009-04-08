#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time any message with ERROR log level"
    echo "usage: $0 LOG_FILE"
    exit
fi

str="ERROR"

./tailgrep.sh "$1" "$str"

#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time feelings suggested by the rule engine"
    echo "It is printed from the beginning of the log"
    echo "Requires DEBUG or finer log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

./tailgrep_from_start.sh "${1}" "Suggesting feeling"

#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time any Rule Engine schema to be executed and from which rule it belongs"
    echo "Requires DEBUG log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

./tailgrep_from_start.sh "${1}" "DEBUG - RuleEngine - cycle"

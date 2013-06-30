#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time all relations created so far and their truth values"
    echo "Requires FINE log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

echo "Please be sure that log level is at FINE"
./tailgrep.sh "${1}" "FINE - RuleEngine - TruthValue relation"

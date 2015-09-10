#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time action schemata suggested by the rule engine"
    echo "Requires DEBUG or finer log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

./tailgrep_from_start.sh "${1}" "Suggesting action"

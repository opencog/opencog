#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time the pet's feelings"
    echo "Requires INFO or finer log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

tail -f "${1}" | grep -f tg_feelings.strings

#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time the action plans attempted to be sent to PAI"
    echo "Requires DEBUG log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

tail -f "${1}" | grep -f tg_action_plan_sent.strings

#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time the procedure selected by the engine rule"
    echo "and the corresponding action plans attempted to be sent to PAI"
    echo "the log messages are printed from the start of the log file"
    echo "Requires DEBUG log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

tail -c+1 -f "${1}" | grep -f tg_action_schema_selected_action_plan_sent.strings

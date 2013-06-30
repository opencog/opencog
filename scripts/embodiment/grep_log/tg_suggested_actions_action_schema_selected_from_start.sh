#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script prints in real-time the action schemata suggested and the chosen schemata by Rule Engine"
    echo "Requires DEBUG log level"
    echo "usage: $0 OAC_LOG_FILE"
    exit
fi

tail -c+1 -f "${1}" | grep -f tg_suggested_actions_action_schema_selected.strings

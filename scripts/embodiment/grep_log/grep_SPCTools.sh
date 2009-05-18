#!/bin/sh
if [ -z "$1" ]; then 
    echo "This script filters all messages used by SPCTools given LS log file."
    echo "Requires DEBUG or finer log level."
    echo "usage: $0 LS_LOG_FILE"
    exit
fi

str="SPCTools"

grep "$str" "$1"

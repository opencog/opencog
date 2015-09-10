#!/bin/sh
if [ -z "$2" ]; then 
    echo "That script greps any line containing STRING_TO_GREP in FILE"
    echo "with FILE a log file filled in real-time"
    echo "usage: $0 FILE STRING_TO_GREP"
    exit
fi
tail -f "${1}" | grep "${2}"

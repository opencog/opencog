#!/bin/sh
if [ -z "$2" ]; then 
    echo "That script greps any line containing STRING_TO_GREP in FILE"
    echo "with FILE a log file filled in real-time"
    echo "it greps from the start of file and then keep greping as the log is filled"
    echo "usage: $0 FILE STRING_TO_GREP"
    exit
fi
#more -f
tail -f -c+1 "${1}" | grep "${2}"

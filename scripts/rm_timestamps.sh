#!/bin/bash
#
# Remove all timestamps from a log file. This convenient when using
# tools just as diff so they ignore timestamp differences.

set -u
# set -x

if [ $# != 1 ]; then
    echo "Description: remove timestamps from a log file"
    echo "Wrong number of arguments"
    echo "Usage: $0 LOG_FILE"
    exit 1
fi

LOG_FILE="$1"

date_re='[[:digit:]]\{4\}-[[:digit:]]\{2\}-[[:digit:]]\{2\}'
time_re='[[:digit:]]\{2\}:[[:digit:]]\{2\}:[[:digit:]]\{2\}:[[:digit:]]\{3\}'
timestamp_re="\[${date_re} ${time_re}\]"
sed "s/^$timestamp_re //" "$LOG_FILE"

#!/bin/bash
#
# Like diff but ignore the timestamps

if [ $# != 2 ]; then
    echo "Description: like diff but ignore logs timestamps"
    echo "Wrong number of arguments"
    echo "Usage: $0 FILE_A FILE_B"
    exit 1
fi

PROG_PATH=$(readlink -f "$0")
PROG_DIR=$(dirname "$PROG_PATH")

fileA=$1
fileB=$2

# remove timestamps of file A
tmp_fileA="$(mktemp)"
"$PROG_DIR/rm_timestamps.sh" "$fileA" > "$tmp_fileA"

# remove timestamps of file B
tmp_fileB="$(mktemp)"
"$PROG_DIR/rm_timestamps.sh" "$fileA" > "$tmp_fileA"

diff "$tmp_fileA" "$tmp_fileB"

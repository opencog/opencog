#!/bin/bash
# This script starts the embodiment "petbrain"

# try to kill existing embodiment session
./stop_embodiment.sh

if [ "$1" == "noclean" ]
then
    # Remove the log and data dirs from last embodiment session
    ./cleanup.sh
fi

./spawner &

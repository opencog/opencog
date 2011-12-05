#!/bin/bash
# This script starts the embodiment "petbrain"

# try to kill existing embodiment session
./stop_embodiment.sh
sleep 2

if [ "$1" != "noclean" ]
then
    # Remove the log and data dirs from last embodiment session
    ./cleanup.sh
fi

./spawner &

# print the occupation of port for spawner, make sure it starts correctly.  
sleep 10
lsof -i:16313 

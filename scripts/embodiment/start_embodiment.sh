#!/bin/bash
# This script starts the embodiment "petbrain"

# try to kill existing embodiment session
./stop_embodiment.sh
sleep 2

echo "Finished stopping processes, cleaning up last session..."

if [ "$1" != "noclean" ]
then
    # Remove the log and data dirs from last embodiment session
    ./cleanup.sh
fi

echo "Cleanup complete, launching spawner process..."

./spawner &

# print the occupation of port for spawner, make sure it starts correctly.  
sleep 10
lsof -i:16313 

echo "Embodiment startup procedure complete, please connect."

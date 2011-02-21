#!/bin/bash
# This script starts the embodiment "petbrain"

# try to kill existing embodiment session
./stop_embodiment

if ($1 != 'noclean')
then
    # Remove the log and data dirs from last embodiment session
    rm -rf /tmp/$USER/Petaverse
fi
# the -p of these commands prevent them from complaining if the directories
# already exist
mkdir -p /tmp/$USER/Petaverse/Logs
mkdir -p /tmp/$USER/Petaverse/PetDatabase

./spawner &

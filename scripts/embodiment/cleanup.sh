#!/bin/bash
# remove log and pet database directories
rm -rf /tmp/$USER/Petaverse/

# the -p of these commands prevent them from complaining if the directories
# already exist
mkdir -p /tmp/$USER/Petaverse/Logs
mkdir -p /tmp/$USER/Petaverse/PetDatabase


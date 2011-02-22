#!/bin/bash

# stop any existing session and reset data and logs
./stop_embodiment.sh
./cleanup.sh

echo "Starting router..."
./router &
sleep 5
echo "Starting LS..."
./learningServer &
sleep 10
echo "Starting MockOACHCTest..."
#comment the following command line
#and uncomment the 3 after if you want to use a debugger
./opcHcTester Fido 16330
#echo ===== you may want to enter the following arguments in the debugger ====
#echo ===== Fido 16330                                                    ====
#nemiver ./opcHcTester Fido 16330

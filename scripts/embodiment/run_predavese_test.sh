#!/bin/tcsh

./stop_embodiment.sh
./cleanup.sh

echo "Start router, please wait..."
../bin/src/Control/MessagingSystem/router &
sleep 5
echo "Start LSMocky, please wait..."
../bin/src/Learning/LearningServer/LSMocky &
sleep 5
echo "Start PredaveseTest.rb, will start OAC..."
./run_predavese_mocky_proxy.rb &
sleep 2
echo "Start OAC, will start OAC..."
../bin/src/Control/OperationalAvatarController/oac 1 16330 &

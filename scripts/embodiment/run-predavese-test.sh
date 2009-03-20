#!/bin/tcsh

./killpb.csh
./cleanup.csh

echo "Start router, please wait..."
../bin/src/Control/MessagingSystem/router &
sleep 5
echo "Start LSMocky, please wait..."
../bin/src/Learning/LearningServer/LSMocky &
sleep 5
echo "Start PredaveseTest.rb, will start OPC..."
./run-predavese-mocky-proxy.rb &
sleep 2
echo "Start OPC, will start OPC..."
../bin/src/Control/OperationalPetController/opc 1 16330 &

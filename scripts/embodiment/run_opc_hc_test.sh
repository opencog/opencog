#!/bin/tcsh

./killpb.csh
./cleanup.csh

echo "Start router, please wait..."
../bin/src/Control/MessagingSystem/router &
sleep 5
echo "Start LS, please wait..."
../bin/src/Learning/LearningServer/learningServer &
sleep 10
echo "Start MockOPCHCTest"
#comment the following command line
#and uncomment the 3 after if you want to use a debugger
../bin/src/Control/OperationalPetController/opcHcTester Fido 16330
#echo ===== enter the following arguments in the debugger ====
#echo ===== Fido 16330                                    ====
#kdbg ../bin/src/Control/OperationalPetController/opcHcTester

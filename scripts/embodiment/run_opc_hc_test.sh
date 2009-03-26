#!/bin/tcsh

./killpb.csh
./cleanup.csh

echo "Start router, please wait..."
./router &
sleep 5
echo "Start LS, please wait..."
./learningServer &
sleep 10
echo "Start MockOPCHCTest"
#comment the following command line
#and uncomment the 3 after if you want to use a debugger
./opcHcTester Fido 16330
#echo ===== you may want to enter the following arguments in the debugger ====
#echo ===== Fido 16330                                                    ====
#nemiver ./opcHcTester Fido 16330

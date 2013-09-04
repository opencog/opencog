#!/bin/sh
echo "Stopping embodiment processes, please wait..."

killall -9 -q oac 
killall -9 -q spawner 
killall -9 -q router 
killall -9 -q learningServer 
killall -9 -q learningServer_CogServer 
killall -9 -q LSMocky 
killall -9 -q pvpSimulator 
killall -9 -q pbTester 

rm -rf *.ipc 

#!/bin/sh

killall -9 oac &> /dev/null
killall -9 spawner &> /dev/null
killall -9 router &> /dev/null
killall -9 learningServer &> /dev/null
killall -9 learningServer_CogServer &> /dev/null
killall -9 LSMocky &> /dev/null
killall -9 pvpSimulator &> /dev/null
killall -9 pbTester &> /dev/null

rm -rf *.ipc &> /dev/null

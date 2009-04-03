#!/bin/tcsh

./killpb.csh
#if ($1 == 'clean') ./cleanup.csh
if ($1 != 'noclean') ./cleanup.csh

./spawner &
# sleep is required to allow all PB services to come up
sleep 10

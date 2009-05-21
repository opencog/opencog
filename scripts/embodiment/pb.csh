#!/bin/tcsh

./killpb.csh
#if ($1 == 'clean') ./cleanup.csh
if ($1 != 'noclean') ./cleanup.csh

./spawner &

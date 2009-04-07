#!/bin/tcsh

if ($1 == 'clean') ./cleanup.csh

wish -f PVPSimulatorInterface.tcl

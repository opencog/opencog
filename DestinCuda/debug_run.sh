#!/bin/sh
echo "Note, the linux x server  can't be running on the graphics card while debuging using cuda-gdb"
echo "On Ubuntu 10 you can log out, then press CTRL-Alt-f1 to switch to a command line session and log in, "
echo "then run  'sudo gdm stop'  to stop the xserver , do some debugging then you can run 'sudo gdm start' and press CTRL-ALT-F8 to get back to your desktop"
echo ""
echo ""
cuda-gdb --args ./destinCuda 00010100000 120 2:3 ./config.xml ../../data/MNISTTraining32 -D D

#!/bin/bash
#
#
# TODO: 
# 1. Automatically detect if user's computer has 32-bit or 64-bit OS
#    installed. And boot Ubuntu precise32 or precise64 accordingly.
#
# Documentation: http://wiki.opencog.org/w/Setup_OpenCog_development_environment_VM_using_Vagrant

echo "==========================================================="
echo "[vagrant-vm-setup.sh] Setting up vagrant VM, please wait..."
echo "==========================================================="

# The "if" decision construct below does one-time-only configuration.
# Unless user manually deletes /home/vagrant/.vagrant_do_not_delete file
# from the VM, the commands in following if block below will run only
# the very first time VM is created (using Vagrant).
if [ ! -f /home/vagrant/.vagrant_do_not_delete ]; then
    sudo apt-get update
    
    # Set grub-pc package on hold: When we do 'apt-get upgrade', 'grub-pc'
    # pops up a ncurses-based dialog to interactively input the disk on 
    # on which to install grub-pc. This beats our aim of a completely
    # automated/unattended install of OpenCog inside a Vagrant VM.
    echo 'grub-pc hold' | sudo dpkg --set-selections
    # Update precise 12.04 
    sudo apt-get -y upgrade
    
    # Install some useful tools available inside VM:
    sudo apt-get -y install screen
    sudo apt-get -y install tmux
    sudo apt-get -y install sysvbanner
    
    # Print OpenCog banner whenever user logs in (vagrant ssh) into VM:
    echo "banner opencog" >> /home/vagrant/.bashrc
    
    # Install package for 'apt-add-repository'
    # For <= 12.04
    sudo apt-get -y install python-software-properties
    # For >= 12.10 (not needed right now because Vagrant supports only 
    # Precise and Lucid versions of Ubuntu):
    #sudo apt-get install software-properties-common
    
    # Install OpenCog dependencies using 'ocpkg' script:
    echo "Downloading ocpkg to setup OpenCog development environment..."
    wget -c https://raw.github.com/opencog/ocpkg/master/ocpkg
    mv ocpkg octool
    chmod +x octool
    echo "Adding apt repositories..."
    sudo ./octool -a
    echo "Installing build dependencies..."
    sudo ./octool -d
    rm ./octool
    
    # All major one-time-only commands (to setup OpenCog development
    # environment) executed above. Let's create a placeholder file so
    # that vagrant does not executes these one-time-only commands next
    # time VM is booted.
    touch /home/vagrant/.vagrant_do_not_delete
    
    # Build OpenCog
    cd /vagrant
    cmake .
    make
fi

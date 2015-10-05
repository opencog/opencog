#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on debian.The script has been tested using docker
# image debian:testing.
# It is provided for those on 32-bit system or don't want to use
# If you encounter an issue don't hesitate to supply a patch on github.

# trap errors
set -e

# Environment Variables
SELF_NAME=$(basename $0)

# Some tools
PACKAGES_TOOLS="
		git \
		python-pip \
		wget \
		sudo \
		"

# Packages for building opencog
PACKAGES_BUILD="
		build-essential \
		cmake \
		cxxtest \
		rlwrap \
		guile-2.0-dev \
		libiberty-dev \
		libicu-dev \
		libbz2-dev \
		cython \
		python-dev \
		python-zmq \
		python-simplejson \
		libboost-date-time-dev \
		libboost-filesystem-dev \
		libboost-math-dev \
		libboost-program-options-dev \
		libboost-regex-dev \
		libboost-serialization-dev \
		libboost-thread-dev \
		libboost-system-dev \
		libjson-spirit-dev \
		libzmq3-dev \
		libtbb-dev \
		binutils-dev \
		libgsl0-dev \
		unixodbc-dev \
		uuid-dev \
		libprotoc-dev \
		protobuf-compiler \
		libsdl-gfx1.2-dev \
		libssl-dev \
		tcl-dev \
		tcsh \
		libfreetype6-dev \
		libatlas-base-dev \
		gfortran \
		"

# Packages required for integrating opencog with other services
PACKAGES_RUNTIME="
		unixodbc \
		odbc-postgresql \
		postgresql-client \
		"

# Template for messages printed.
message() {
echo -e "\e[1;34m[$SELF_NAME] $MESSAGE\e[0m"
}

# Install cogutils
install_cogutil(){
MESSAGE="Installing cogutils...." ; message
cd /tmp/
wget https://github.com/opencog/cogutils/archive/master.tar.gz
tar -xvf master.tar.gz
cd cogutils-master/
mkdir build
cd build/
cmake ..
make -j$(nproc)
sudo make install
cd ../..
rm -rf master.tar.gz cogutils-master/
}

# Install Python Packages
install_python_packages(){
MESSAGE="Installing python packages...." ; message
cd /tmp
wget https://raw.githubusercontent.com/opencog/opencog/master/opencog/python/requirements.txt
sudo pip install -U -r /tmp/requirements.txt
rm requirements.txt
}

# Install AtomSpace
install_atomspace(){
MESSAGE="Installing atomspace...." ; message
cd /tmp/
wget https://github.com/opencog/atomspace/archive/master.tar.gz
tar -xvf master.tar.gz
cd atomspace-master/
mkdir build
cd build/
cmake ..
make -j$(nproc)
sudo make install
cd ../..
rm -rf master.tar.gz atomspace-master/
}

# Function for installing all required dependenceis for building OpenCog,
# as well as dependencies required for running opencog with other services.
install_dependencies() {
MESSAGE="Updating Package db...." ; message
apt-get update

MESSAGE="Installing OpenCog build dependencies...." ; message
if ! (apt-get -y install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS); then
  MESSAGE="Error installing some of the dependencies... :( :("  ; message
  exit 1
fi
install_python_packages
install_cogutil
install_atomspace
}

# Main Program
install_dependencies

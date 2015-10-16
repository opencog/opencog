#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on archlinux.The script has been tested using docker
# image base/archlinux:latest & l3iggs/archlinux
# It is provided for those on 32-bit system or don't want to use
# If you encounter an issue don't hesitate to supply a patch on github.

# trap errors
set -e

# Environment Variables
SELF_NAME=$(basename $0)

PACKAGES_TOOLS="
 		git \
		python-pip \
		wget \
		sudo \
		pkg-config \
		"

# Packages for building opencog
PACKAGES_BUILD="
		gcc \
		make \
		cmake \
		cxxtest \
		rlwrap \
		guile \
		icu
		bzip2 \
		cython \
		python2 \
		python2-pyzmq \
		python2-simplejson \
		boost \
        zeromq \
        intel-tbb \
        binutils \
        gsl \
        unixodbc \
        protobuf \
        protobuf-c \
		sdl_gfx \
		openssl \
		tcl \
		tcsh \
		freetype2 \
		blas \
		lapack \
		gcc-fortran \
		"

PACKAGES_RUNTIME="
		unixodbc \
		psqlodbc \
		libpqxx \
		"

# Template for messages printed.
message() {
echo -e "\e[1;34m[$SELF_NAME] $MESSAGE\e[0m"
}

# Install  json-spirit (4.05)
install_json_spirit(){
MESSAGE="Installing json-spirit library...." ; message
cd /tmp
export BOOST_ROOT=/usr/include/boost/
wget http://http.debian.net/debian/pool/main/j/json-spirit/json-spirit_4.05.orig.tar.gz
tar -xvf json-spirit_4.05.orig.tar.gz
cd json_spirit_v4_05
mkdir build
cd build/
cmake ..
make -j$(nproc)
sudo make install
cd ../..
rm -rf json-spirit_4.05.orig.tar.gz json_spirit_v4_05
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
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  pacman -S --noconfirm $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS; then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
install_python_packages
install_json_spirit
install_cogutil
install_atomspace
}

# Main Program
install_dependencies

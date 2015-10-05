#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on openSUSE.The script has been tested using docker
# image opensuse:13.2
# It is provided for those on 32-bit system or don't want to use
# If you encounter an issue don't hesitate to supply a patch on github.

# TODO Make it work

# trap errors
set -e

# Environment Variables
SELF_NAME=$(basename $0)

# Some tools
PACKAGES_TOOLS="
		git \
		python-pip \
		wget \
		"

# Packages for building opencog
# FIXME cxxtest and tbb are not installaed
PACKAGES_BUILD="
		gcc \
		make \
		cmake \
		cxxtest \
		rlwrap \
		guile \
		libicu-devel \
		libzip2 \
		python-Cython \
		python-devel \
		python-pyzmq \
		python-simplejson \
		boost-devel \
		libzmq3 \
		zeromq-devel \
		binutils-devel \
		libgsl0 gsl-devel \
		unixodbc-devel \
		uuid-devel \
		libprotobuf-c-devel \
		libSDL_gfx-devel \
		libssl27 \
		tcl  \
		tcsh \
		freetype2-devel \
		libatlas3 \
		gcc-fortran \
		"

# Packages required for integrating opencog with other services
PACKAGES_RUNTIME="
		unixODBC-devel \
		psqlODBC \
		postgresql \
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
# FIXME Haven't figured out which package is making this check fail.
if ! (zypper --no-refresh install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS);
then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
install_json_spirit
install_python_packages
install_cogutil
install_atomspace
}

# Main Program
install_dependencies

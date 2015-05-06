#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on fedora.The script has been tested using docker
# image fedora:21.

# trap errors
set -e

# Environment Variables
SELF_NAME=$(basename $0)

# Some tools
PACKAGES_TOOLS="
		git \
		python-pip \
		wget \
		tar \
		sudo \
			"

# Packages for building opencog
PACKAGES_BUILD="
		make \
		gcc \
		gcc-c++ \
		cmake \
		cxxtest \
		rlwrap \
		guile-devel \
		libicu-devel \
		bzip2-devel \
		Cython \
		python-devel \
		python-zmq \
		boost-devel \
		zeromq-devel \
		tbb-devel \
		binutils-devel \
		gsl-devel \
		unixODBC-devel \
		xerces-c-devel \
		uuid-devel \
		protobuf-c \
		protobuf-compiler
		gnutls-devel \
		expat-devel \
		SDL_gfx-devel \
		openssl-devel \
		tcl-devel \
		tcsh \
		freetype-devel \
		atlas-devel \
		gcc-gfortran \
		"

# Packages required for integrating opencog with other services
PACKAGES_RUNTIME="
		unixODBC \
		postgresql-odbc \
		postgresql-devel \
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

# Install Python Packages
install_python_packages(){
MESSAGE="Installing python packages...." ; message
cd /tmp
wget https://raw.githubusercontent.com/opencog/opencog/master/opencog/python/requirements.txt
sudo pip install -U -r /tmp/requirements.txt
rm requirements.txt
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
yum updateinfo

MESSAGE="Installing OpenCog build dependencies...." ; message
if !  (yum -y install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS); then
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

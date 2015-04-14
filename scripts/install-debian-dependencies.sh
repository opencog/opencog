#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on debian.The script has been tested using docker
# image debian:testing.
# To install extra python-dependencies use pip and the requirements
# file @ https://github.com/opencog/opencog/blob/master/opencog/python/requirements.txt

# trap errors
set -e

# Some tools
PACKAGES_FETCH="
		git \
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
		python-matplotlib \
		python-nose \
		python-numpy \
		python-scipy \
		python-mock \
		python-simplejson \
		python-pyparsing \
		python-dateutil \
		python-six \
		python-tornado \
		python-wsgiref \
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
		libxerces-c-dev \
		uuid-dev \
		libprotoc-dev \
		protobuf-compiler \
		libcurl4-gnutls-dev \
		libexpat1-dev \
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

# Function for installing all required dependenceis for building OpenCog,
# as well as dependencies required for running opencog with other services.
install_dependencies() {
MESSAGE="Updating Package db...." ; message
apt-get update

MESSAGE="Installing OpenCog build dependencies...." ; message
if !  apt-get --no-upgrade --assume-yes install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_FETCH; then
  MESSAGE="Error installing some of the dependencies... :( :("  ; message
  exit 1
fi
}

# Main Program
install_dependencies

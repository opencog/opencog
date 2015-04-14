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
			"

# Packages for building opencog
# FIXME fedora version of libjson-spirit-dev on ubuntu not found yet.
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

# Path to requirements file
PACKAGES_PYTHON="../opencog/python/requirements.txt"

# Template for messages printed.
message() {
echo -e "\e[1;34m[$SELF_NAME] $MESSAGE\e[0m"
}

# Function for installing all required dependenceis for building OpenCog,
# as well as dependencies required for running opencog with other services.
install_dependencies() {
MESSAGE="Updating Package db...." ; message
yum updateinfo

MESSAGE="Installing OpenCog build dependencies...." ; message
if !  (yum -y install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS && \
		pip install -U -r $PACKAGES_PYTHON); then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
}

# Main Program
install_dependencies

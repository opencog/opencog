#!/bin/bash
#
# This script is used for installing the dependencies required for
# building opencog on fedora.The script has been tested using docker
# image fedora:21.
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
        tar \
        sudo \
		curl \
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
        uuid-devel \
        protobuf-c \
        protobuf-compiler \
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
make -j"$(nproc)"
sudo make install
cd ../..
rm -rf json-spirit_4.05.orig.tar.gz json_spirit_v4_05
}

# Install Python Packages
install_opencog_python_packages(){
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
make -j"$(nproc)"
sudo make install
cd ../..
rm -rf master.tar.gz cogutils-master/
}

# Install Link-Grammar
install_link_grammar(){
MESSAGE="Installing Link-Grammar...." ; message
cd /tmp/
# cleaning up remnants from previous install failures, if any.
rm -rf link-grammar-5.*/
wget -r --no-parent -nH --cut-dirs=2 http://www.abisource.com/downloads/link-grammar/current/
tar -zxf current/link-grammar-5*.tar.gz
rm -r current
cd link-grammar-5.*/
mkdir build
cd build
../configure
make -j"$(nproc)"
sudo make install
sudo ldconfig
cd /tmp/
rm -rf link-grammar-5.*/
cd $CURRENT_DIR
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
make -j"$(nproc)"
sudo make install
cd ../..
rm -rf master.tar.gz atomspace-master/
}

# Function for installing all required dependenceis for building OpenCog,
# as well as dependencies required for running opencog with other services.
install_dependencies() {
MESSAGE="Updating Package db...." ; message
dnf updateinfo

MESSAGE="Installing OpenCog build dependencies...." ; message
if !  (sudo dnf -y install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_TOOLS)
then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
install_json_spirit
}

# Install Haskell Dependencies
install_haskell_dependencies(){
if ! is_x68_64_fedora_22; then
MESSAGE="Installing haskell dependencies in user space...." ; message
# Install stack.
curl -sSL https://s3.amazonaws.com/download.fpcomplete.com/fedora/22/fpco.repo | sudo tee /etc/yum.repos.d/fpco.repo
sudo dnf -y install stack
fi

# Notes
# 1. Stack setup must me run in user space:
#    "stack setup" looks for proper ghc version in the system according to the
#    information provided by stack.yaml. If it is not installed, it attempts to
#    install proper ghc version on user space (~/.stack/...). Because of that,
#    it must not be run as root.

# 2. Difference b/n .cabal and stack.yaml:
#    The .cabal file contains package metadata, in this case
#    "opencog-atomspace.cabal" contains information of the opencog-atomspace
#    package (autor, license, dependencies, etc.). The stack.yaml file contains
#    configuration options for the stack building tool, we use it to set the
#    proper "long term support" snapshot that we are using, which determines the
#    proper ghc version to use, etc. In this case, it doesn't make sense to
#    require the .cabal file, because we are not using that information to build
#    the hscolour package, but it looks like stack always looks for a .cabal
#    file when building, even though, in this case, it doesn't use it.
if [ "$EUID" -ne 0 ] ; then
    cd /tmp
    wget https://raw.githubusercontent.com/opencog/atomspace/master/opencog/haskell/stack.yaml
    wget https://raw.githubusercontent.com/opencog/atomspace/master/opencog/haskell/opencog-atomspace.cabal
    stack setup

    # hscolour is necessary for haddock documentation.
    stack build hscolour --copy-bins
    rm stack.yaml opencog-atomspace.cabal
    cd $CURRENT_DIR
else
    echo "Please run without sudo. Stack need to be run in non-root user space."
fi
}

usage() {
echo "Usage: $SELF_NAME OPTION"
echo " -d Install base/system build dependencies"
echo " -p Install opencog python build dependencies"
echo " -s Install haskell build dependencies in user space. Don't use sudo"
echo " -c Install Cogutils"
echo " -a Install Atomspace"
echo " -l Install Link Grammar"
echo " -h This help message"
}

# Main Program
if [ $# -eq 0 ] ; then NO_ARGS=true ; fi

while getopts "dpcalsh" flag ; do
    case $flag in
      d)    INSTALL_DEPENDENCIES=true ;; #base development packages
      p)    INSTALL_OPENCOG_PYTHON_PACKAGES=true ;;
      c)    INSTALL_COGUTIL=true ;;
      a)    INSTALL_ATOMSPACE=true ;;
      l)    INSTALL_LINK_GRAMMAR=true ;;
      s)    HASKELL_STACK_SETUP=true;;
      h)    usage ;;
      \?)    usage ;;
      *)  UNKNOWN_FLAGS=true ;;
    esac
done

if [ $INSTALL_DEPENDENCIES ] ; then install_dependencies ; fi
if [ $HASKELL_STACK_SETUP ] ; then install_haskell_dependencies ; fi
if [ $INSTALL_OPENCOG_PYTHON_PACKAGES ] ; then
    install_opencog_python_packages
fi
if [ $INSTALL_COGUTIL ] ; then install_cogutil ; fi
if [ $INSTALL_ATOMSPACE ] ; then install_atomspace ; fi
if [ $INSTALL_LINK_GRAMMAR ] ; then install_link_grammar ; fi
if [ $UNKNOWN_FLAGS ] ; then usage ; fi
if [ $NO_ARGS ] ; then usage ; fi

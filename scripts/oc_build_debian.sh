# trap errors
set -e

PATH_PREFIX=/usr/local
CURRENT_DIR=$(pwd)

HOST_SOURCE_BRANCH=$PATH_PREFIX/src/opencog  
HOST_BUILD_DIR=/tmp/opencog_build

SLEEP_TIME=0

SELF_NAME=$(basename $0)
TOOL_NAME=octool

PROCESSORS=$(grep "^processor" /proc/cpuinfo | wc -l)
MAKE_JOBS=$(($PROCESSORS+0))


VERBOSE="-v"				#for mount, umount, rm, etc.
QUIET="" 				#for apt-get



PACKAGES_TOOLS="	
		squashfs-tools \
		genisoimage \
		"

PACKAGES_FETCH="	
		git \
			"

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
		#liblua5.1-0-dev \
		#libxmlrpc-c3-dev \
		#python-flask \
		#python-flask-restful \

PACKAGES_RUNTIME="	
		unixodbc \
		odbc-postgresql \
		postgresql-client \
		"

# SECTION
message() {
echo -e "\e[1;34m[$SELF_NAME] $MESSAGE\e[0m"
}

PACKAGE_TYPE=$DEFAULT_PACKAGE_TYPE


debug() {
MESSAGE="Dropping to debugging chroot prompt..." ; message
chroot $LIVE_SQUASH_UNION /bin/bash -l 
}

exit_trap() {
if [ "$SELF_NAME" != "$TOOL_NAME" ] ; then
  MESSAGE="Exiting $SELF_NAME normally..." ; message
fi
}

quit_trap() {
if [ "$SELF_NAME" != "$TOOL_NAME" ] ; then
  MESSAGE="Exiting $SELF_NAME by request..." ; message
fi
}

error_trap() {
if [ "$SELF_NAME" != "$TOOL_NAME" ] ; then
  MESSAGE="Error trapped while running $SELF_NAME." ; message
fi
}

trap error_trap ERR 
trap exit_trap  EXIT 
trap quit_trap  INT HUP QUIT TERM

# SECTION 4: Function Definitions

add_repositories() {
MESSAGE="Adding\Updating software repositories..." ; message
   apt-get $QUIET --assume-yes update 
}


install_dependencies() {
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  apt-get $QUIET --no-upgrade --assume-yes install $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_FETCH; then
  MESSAGE="Please enable 'universe' repositories and re-run this script."  ; message
  exit 1
fi
}

update_opencog_source() { 
if apt-get --no-upgrade --assume-yes $QUIET install $PACKAGES_FETCH ; then
  echo -n
else
  MESSAGE="Please enable 'universe' repositories and re-run this script."  ; message
exit 1
fi
OPENCOG_SOURCE_DIR=$LIVE_SOURCE_BRANCH
mkdir -p $OPENCOG_SOURCE_DIR || true
if [ ! "$(ls -A $OPENCOG_SOURCE_DIR/.git)" ]; then
  MESSAGE="Fetching OpenCog source at $OPENCOG_SOURCE_DIR..." ; message
  git clone https://github.com/opencog/opencog $OPENCOG_SOURCE_DIR
else
  if [ $UPDATE_OPENCOG ] ; then
    MESSAGE="Updating OpenCog source at $OPENCOG_SOURCE_DIR..." ; message
    cd $OPENCOG_SOURCE_DIR
    git pull 
    cd -
  fi
fi
}

build_opencog() {
mkdir -p -v $HOST_BUILD_DIR || true
cd $HOST_BUILD_DIR
MESSAGE="cmake $LIVE_SOURCE_BRANCH" ; message
cmake $HOST_SOURCE_BRANCH
MESSAGE="make -j$MAKE_JOBS" ; message
make -j$MAKE_JOBS
make test



# SECTION 5: Main Program (MAIN main)
add_repositories; 
install_dependencies
update_opencog_source
build_opencog


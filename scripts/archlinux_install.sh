# SECTION 1 Declare vars and collect environment info.
# trap errors 
set -e

PATH_PREFIX=/usr/local
CURRENT_DIR=$(pwd)

HOST_SOURCE_BRANCH=$CURRENT_DIR/opencog/src  
HOST_BUILD_DIR=$CURRENT_DIR/opencog/build

# SECTION 2 Assemble install info.
SELF_NAME=$(basename $0)

# get processor count
PROCESSORS=$(grep "^processor" /proc/cpuinfo | wc -l)

# make job count equal to processor count
MAKE_JOBS=$(($PROCESSORS+0))

VERBOSE="-v"				#for mount, umount, rm, etc.
QUIET="-q" 				

PACKAGES_FETCH=" git "

PACKAGES_ADMIN=" epiphany-browser "

PACKAGES_BUILD="
		base-devel	
		cmake \
		cxxtest \
		rlwrap \
		guile \
		icu
		bzip2 \
		cython \
		python \
		python-pyzmq \
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
		python2-pydot \
		boost-libs \
                zeromq \
                intel-tbb \
                binutils \
                gsl \
                unixodbc \
                xerces-c \
                libutil-linux \
                protobuf \
                protobuf-c \
                gnurl \
                curl \
                expat \

		sdl_gfx \
		openssl \
		tcl \
		tcsh \
		freetype2 \
		blas \
		gcc-fortran \
		python-flask \
		"
		#liblua5.1-0-dev \
		#libxmlrpc-c3-dev \
		#python-flask-restful \

PACKAGES_RUNTIME="	
		unixodbc \
		psqlodbc \
		libpqxx \
		"
# SECTION 3 Debug info routines.
message() {
echo -e "\e[1;34m[$SELF_NAME] $MESSAGE\e[0m"
}

PACKAGE_TYPE=$DEFAULT_PACKAGE_TYPE

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
  debug
fi
}

# execute error_trap on signal ERR
trap error_trap ERR 
# execute exit_trap on signal EXIT
trap exit_trap  EXIT
# execute quit_trap on signal INT HUP QUIT TERM
trap quit_trap  INT HUP QUIT TERM

# SECTION 4: Function Definitions
add_repositories() {
MESSAGE="Adding software repositories..." ; message
for REPO in $REPOSITORIES ; do 
#Install a 'remote' package (not from a repository stated in pacman's configuration files)
  pacman -U $REPO
done
# Update package database 
  pacman -Sy
}


install_dependencies() {
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  pacman -S $QUIET $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_FETCH; then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
}

update_opencog_source() { 
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
mkdir -p -v $LIVE_BUILD_DIR || true
cd $HOST_BUILD_DIR
MESSAGE="cmake $HOST_SOURCE_BRANCH" ; message
cmake $HOST_SOURCE_BRANCH
MESSAGE="make -j$MAKE_JOBS" ; message
make -j$MAKE_JOBS

if [ $TEST_OPENCOG ] ; then 
  make test
fi

}
# SECTION 5: Main Program (MAIN main)
install_dependencies
update_opencog_source
build_opencog

# trap errors 
set -e

PATH_PREFIX=/usr/local
CURRENT_DIR=$(pwd)

HOST_SOURCE_BRANCH=$CURRENT_DIR/opencog/src
HOST_BUILD_DIR=$CURRENT_DIR/opencog/build

SELF_NAME=$(basename $0)
TOOL_NAME=octool

# get processor count
PROCESSORS=$(grep "^processor" /proc/cpuinfo | wc -l)

# make job count equal to processor count
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

PACKAGE_GROUPS_BUILD="
			
			"

PACKAGES_BUILD="	
		make \
		automake \
		gcc \
		gcc-c++ \
		cmake \
		cxxtest \
		rlwrap \
		guile-devel \
		binutils-devel \
		libicu-devel \
		bzip2-devel \
		Cython \
		python-devel \
		python-zmq \
		python-matplotlib \
		python-nose \
		numpy \
		scipy \
		python-mock \
		python-simplejson \
		pyparsing \
		python-dateutil \
		python-six \
		python-tornado \
		python-wsgiref \
		boost-devel \
                zeromq-devel \
                tbb-devel \
                gsl-devel \
                unixODBC-devel \
                xerces-c-devel \
		uuid-devel \
		protobuf-c \
                protobuf-devel \
		protobuf-compiler
                gnutls-devel \
                curl \
                expat-devel \

		SDL_gfx-devel \
		openssl-devel \
		tcl-devel \
		tcsh \
		freetype-devel \
		atlas-devel \
		libgfortran gcc-gfortran \
		"
		#liblua5.1-0-dev \
		#libxmlrpc-c3-dev \
		#python-flask \
		#python-flask-restful \

PACKAGES_RUNTIME="	
		unixODBC \
		postgresql-odbc \
		postgresql-devel \
		"

# SECTION
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
  MESSAGE="Cleanup will run after debug." ; message
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
MESSAGE="Adding/Updating software repositories..." ; message
# Update package database and upgrade
  yum distro-sync
}


install_dependencies() {
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  yum install $QUIET $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_FETCH; then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
}

update_opencog_source() { 
yum install $QUIET $PACKAGES_FETCH ; 
  echo -n
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
MESSAGE="cmake $HOST_SOURCE_BRANCH" ; message
cmake $LIVE_SOURCE_BRANCH
MESSAGE="make -j$MAKE_JOBS" ; message
make -j$MAKE_JOBS
make test
}

# SECTION 5: Main Program (MAIN main)
install_dependencies
update_opencog_source
build_opencog


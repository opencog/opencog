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
QUIET="" 				

PACKAGES_FETCH=" git "

PACKAGES_ADMIN=" epiphany-browser "

PACKAGES_PATTERN_INSTALL=" devel_C_C++ "

PACKAGES_SOURCE_INSTALL=" tbb "

PACKAGES_BUILD="
		devel_C_C++ \
		devel_tcl \
		zeromq-devel \
		cmake \
		rlwrap \
		guile libguile-2_0-22 \
		libicu-devel \
		libzip2 \
		python-Cython \
		python-devel \
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
		python-wsgiref \
		python-pydot \
		boost-devel \
                libzmq3 \
                binutils-devel \
                libgsl0 gsl-devel \
                unixodbc-devel \
                libxerces-c-devel \
                uuid-devel \
                libprotoc8 protobuf-c libprotobuf-c-devel \
                libcurl4 \
                curl \
                libexpat-devel  \
		libSDL_gfx-devel \
		libssl27 \
		tcl-devel  \
		tcsh \
		libfreetype6 freetype2-devel \
		blas-devel libblas3  \
		gcc-fortran libgfortran3 \
		python-Flask \
		tbb \
		"
		#liblua5.1-0-dev \
		#libxmlrpc-c3-dev \
		#python-flask-restful \

PACKAGES_RUNTIME="	
		unixODBC-devel \
		psqlODBC \
		postgresql \
		"
REPOSITORIES="http://download.opensuse.org/repositories/devel:libraries:c_c++/openSUSE_Factory_ppc/devel:libraries:c_c++.repo"
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
  zypper addrepo $REPO
done
# Update package database 
  zypper refresh
}


install_dependencies() {
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  zypper --no-refresh install $QUIET $PACKAGES_BUILD $PACKAGES_RUNTIME 
$PACKAGES_FETCH; 
then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
}

update_opencog_source() { 
OPENCOG_SOURCE_DIR=$LIVE_SOURCE_BRANCH
mkdir -p $OPENCOG_SOURCE_DIR || true
if [ ! "$(ls -A $OPENCOG_SOURCE_DIR/.git)" ]; then
  MESSAGE="Fetching OpenCog source at $OPENCOG_SOURCE_DIR..." ; message
#  git clone https://github.com/opencog/opencog $OPENCOG_SOURCE_DIR
else
  if [ $UPDATE_OPENCOG ] ; then
    MESSAGE="Updating OpenCog source at $OPENCOG_SOURCE_DIR..." ; message
    cd $OPENCOG_SOURCE_DIR
 #   git pull 
    cd -
  fi
fi
}

pattern_install()
{
	zypper install -t pattern $PACKAGES_PATTERN_INSTALL
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

# For Intel's Threading Block Library (TBB)

# source install 
zypper si tbb
tar -xvzf "/usr/src/packages/SOURCES/tbb*.tar.gz" 
make target=linux $MAKE_JOBS
pattern_install
add_repositories
install_dependencies
update_opencog_source
build_opencog

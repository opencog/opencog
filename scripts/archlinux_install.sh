# trap errors 
set -e

PATH_PREFIX=/usr/local
CURRENT_DIR=$(pwd)

if [ "$USER" == "root" ] ; then
 HOST_SOURCE_BRANCH=$PATH_PREFIX/src/opencog  
 HOST_BUILD_DIR=/tmp/opencog_build
else
 HOST_SOURCE_BRANCH=$CURRENT_DIR/opencog/src  
 HOST_BUILD_DIR=$CURRENT_DIR/opencog/build
fi

SLEEP_TIME=0

DEFAULT_PACKAGE_TYPE=local		
DEFAULT_ADD_REPOSITORIES=true
DEFAULT_INSTALL_DEPENDENCIES=true	
DEFAULT_UPDATE_OPENCOG=true		
#DEFAULT_BUILD_OPENCOG=true		
#DEFAULT_TEST_OPENCOG=true		


SELF_NAME=$(basename $0)
TOOL_NAME=octool

# get processor count
PROCESSORS=$(grep "^processor" /proc/cpuinfo | wc -l)

# make job count equal to processor count
MAKE_JOBS=$(($PROCESSORS+0))


LIVE_SOURCE_BRANCH=$HOST_SOURCE_BRANCH
LIVE_BUILD_DIR=$HOST_BUILD_DIR

VERBOSE="-v"				#for mount, umount, rm, etc.
QUIET="" 				#for apt-get

LIVE_DESKTOP_SOURCE="OpenCog Source Code"


PACKAGES_TOOLS="	
		squashfs-tools \
		libisoburn \
		libisofs \
		"

PACKAGES_FETCH="	
		git \
			"
		#bzr-rewrite \

PACKAGES_ADMIN="epiphany-browser \
		"

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
		"
		#liblua5.1-0-dev \
		#libxmlrpc-c3-dev \
		#python-flask \
		#python-flask-restful \

PACKAGES_RUNTIME="	
		unixodbc \
		psqlodbc \
		libpqxx \
		"

PACKAGES_DOC="		
		
		glib2-docs \
		libpango1.0-doc \
		libgtk-3-doc \
		texlive-latex-base-doc \
		python-doc \
		devhelp-common \
		texlive-doc-base \
		doxygen \
 		dot2tex \
		"

PACKAGES_EXDEV="	
		autoconf automake autotools-dev \
		blt \
		comerr-dev \
		dpkg-dev \
		emacsen-common \
		fakeroot \
		gettext \
		gdb \
		g++-4.6 \
		gcc-4.6 \
		krb5-multidev \
		libc6-dev \
		libdpkg-perl \
		libgcrypt11-dev \
		libglade2-0 \
		libgnutls-dev \
		libgpg-error-dev \
		libgssrpc4 \
		libidn11-dev \
		libalgorithm-diff-perl \
		libalgorithm-diff-xs-perl \
		libalgorithm-merge-perl \
		libkadm5clnt-mit8 \
		libkadm5srv-mit8 \
		libkrb5-dev \
		libldap2-dev \
		libltdl-dev \
		libstdc++-dev \
		libtasn1-3-dev \
		libtimedate-perl \
		libtool \
		libunistring0 \
		libxss1 \
		make \
		manpages-dev \
		m4 \
		patch \
		python-glade2 \
		python-tk \
		tcl8.5 \
		tk8.5 \
		ttf-lyx \
		zlib1g-dev \
		python-bzrlib \
		linux-headers-generic \
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
MESSAGE="Adding software repositories..." ; message
for REPO in $REPOSITORIES ; do 
#Install a 'remote' package (not from a repository stated in pacman's configuration files)
  pacman -U $REPO
done
# Update package database and upgrade
  pacman -Syu
}


install_dependencies() {
MESSAGE="Installing OpenCog build dependencies...." ; message
if !  pacman -S $QUIET $PACKAGES_BUILD $PACKAGES_RUNTIME $PACKAGES_FETCH; then
  MESSAGE="Error installing some of dependencies... :( :("  ; message
  exit 1
fi
}

update_opencog_source() { 
if pacman -S $QUIET $PACKAGES_FETCH ; then
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
mkdir -p -v $LIVE_BUILD_DIR || true
cd $LIVE_BUILD_DIR
MESSAGE="cmake $LIVE_SOURCE_BRANCH" ; message
cmake $LIVE_SOURCE_BRANCH
MESSAGE="make -j$MAKE_JOBS" ; message
make -j$MAKE_JOBS

if [ $TEST_OPENCOG ] ; then 
  make test
fi

case $PACKAGE_TYPE in
  min)	MESSAGE="Installing OpenCog..." ; message
 	make install; exit 0;;
  demo)	MESSAGE="Installing OpenCog..." ; message
  	make install; exit 0;;
  dev)	exit 0;;
esac

}




# SECTION 5: Main Program (MAIN main)

add_repositories
install_dependencies
update_opencog_source
build_opencog


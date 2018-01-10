#!/bin/sh
# 
# Run this before configure
#
# This file blatantly ripped off from subversion.
#
# Note: this dependency on Perl is fine: only developers use autogen.sh
#       and we can state that dev people need Perl on their machine
#
# TODO: make sure that ac_pkg_swig.m4 gets installed in m4 directory ...

rm -f autogen.err

automake --version | perl -ne 'if (/\(GNU automake\) (([0-9]+).([0-9]+))/) {print; if ($2 < 1 || ($2 == 1 && $3 < 4)) {exit 1;}}'

if [ $? -ne 0 ]; then
    echo "Error: you need automake 1.4 or later.  Please upgrade."
    exit 1
fi

if test ! -d `aclocal --print-ac-dir 2>> autogen.err`; then
  echo "Bad aclocal (automake) installation"
  exit 1
fi

libtoolize --force --copy || {
    echo "error: libtoolize failed"
    exit 1
}

# Produce aclocal.m4, so autoconf gets the automake macros it needs
# 
case `uname` in
    CYGWIN*)
        include_dir='-I m4' # Needed for Cygwin only.
esac
echo "Creating aclocal.m4: aclocal $include_dir $ACLOCAL_FLAGS"

aclocal $include_dir $ACLOCAL_FLAGS 2>> autogen.err

# Produce all the `GNUmakefile.in's and create neat missing things
# like `install-sh', etc.
# 
echo "automake --add-missing --copy --foreign"

automake --add-missing --copy --foreign 2>> autogen.err || {
    echo ""
    echo "* * * warning: possible errors while running automake - check autogen.err"
    echo ""
}

# If there's a config.cache file, we may need to delete it.  
# If we have an existing configure script, save a copy for comparison.
if [ -f config.cache ] && [ -f configure ]; then
  cp configure configure.$$.tmp
fi

# Produce ./configure
# 
echo "Creating configure..."

autoconf 2>> autogen.err || {
    echo ""
    echo "* * * warning: possible errors while running automake - check autogen.err"
    echo ""
}

run_configure=true
for arg in $*; do
    case $arg in
        --no-configure)
            run_configure=false
            ;;
        *)
            ;;
    esac
done

if $run_configure; then
    mkdir -p build
    cd build
    ../configure --enable-maintainer-mode "$@"
    echo
    echo "Now type 'make' to compile link-grammar."
else
    echo
    echo "Now run 'configure' and 'make' to compile link-grammar."
fi

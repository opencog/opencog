OpenCog Utilities
=================

The OpenCog utilities is a miscellaneous collection of C++ utilities
use for typical programming tasks in multiple OpenCog projects.

The main project site is at http://opencog.org

Prerequisites
-------------
To build the OpenCog utilities, the packages listed below are required. With a
few exceptions, most Linux distributions will provide these packages. Users of
Ubuntu 14.04 "Trusty Tahr" may use the dependency installer at scripts/octool.
Users of any version of Linux may use the Dockerfile to quickly build a 
container in which OpenCog will be built and run.

###### boost
> C++ utilities package
> http://www.boost.org/ | libboost-dev

###### cmake
> Build management tool; v2.8 or higher recommended.
> http://www.cmake.org/ | cmake

###### cxxtest
> Test framework
> http://cxxtest.sourceforge.net/ | https://launchpad.net/~opencog-dev/+archive/ppa
> Currently, opencog requires cxxtest version 3, and is not compatible
  with version 4.

Building CogUtils
-----------------
Peform the following steps at the shell prompt:
```
    cd to project root dir
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make
```
Libraries will be built into subdirectories within build, mirroring the
structure of the source directory root. The flag -DCMAKE_BUILD_TYPE=Release
results in binaries that are optimized for for performance; ommitting
this flag will result in faster builds, but slower executables.


Unit tests
----------
To build and run the unit tests, from the ./build directory enter (after
building opencog as above):
```
    make test
```


Install
-------
After building, you MUST install the utilities!
```
    sudo make install
```

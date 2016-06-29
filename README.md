OpenCog
=======

master:
[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-opencog-master)](http://61.92.69.39:8080/job/ci-opencog-master)
stable:
[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-opencog-stable)](http://61.92.69.39:8080/job/ci-opencog-stable)

OpenCog is a framework for developing AI systems, especially appropriate
for integrative multi-algorithm systems, and artificial general intelligence
systems.  Though much work remains to be done, it currently contains a
functional core framework, and a number of cognitive agents at varying levels
of completion, some already displaying interesting and useful functionalities
alone and in combination.

The main project site is at http://opencog.org

An interactive tutorial for getting started is available at:
https://github.com/opencog/opencog/blob/master/TUTORIAL.md

For platform dependent instruction on dependencies and building the code as
well as other options for setting up development environment more details are
found at: http://wiki.opencog.org/wikihome/index.php/Building_OpenCog

Prerequisites
-------------
To build and run OpenCog, the packages listed below are required.
With a few exceptions, most Linux distributions will provide these
packages. Users of Ubuntu 14.04 "Trusty Tahr" may use the dependency
installer at `/scripts/octool`.  Users of any version of Linux may
use the Dockerfile to quickly build a container in which OpenCog will
be built and run.

###### cogutil
> Common OpenCog C++ utilities
> http://github.com/opencog/cogutils
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### atomspace
> OpenCog Atomspace database and reasoning engine
> http://github.com/opencog/atomspace
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

Optional Prerequisites
----------------------
The following packages are optional. If they are not installed, some
optional parts of OpenCog will not be built.  The CMake command, during
the build, will be more precise as to which parts will not be built.

###### Link Grammar
> Natural Language Parser for English, Russian, other languages.
> Required for experimental Viterbi parser.
> http://www.abisource.com/projects/link-grammar/

###### MOSES
> MOSES Machine Learning
> http://github.com/opencog/moses
> It uses exactly the same build proceedure as this pakcage. Be sure
  to `sudo make install` at the end.

###### OpenGL
> Open Graphics Library
> Used by opencog/spatial/MapTool
> http://www.opengl.org
> Commonly provided with your video card driver

###### SDL
> Simple DirectMedia Layer
> Used by opencog/spatial/MapTool
> http://www.libsdl.org | libsdl1.2-dev

###### SDL_gfx
> Simple DirectMedia Layer extension
> Used by opencog/spatial/MapTool
> http://www.ferzkopp.net/joomla/content/view/19/14/ | libsdl-gfx1.2-dev

###### Threading Building Blocks
> C++ template library for parallel programming
> https://www.threadingbuildingblocks.org/download | libtbb-dev

###### ZeroMQ (version 3.2.4 or higher)
> Asynchronous messaging library
> http://zeromq.org/intro:get-the-software | libzmq3-dev

Building OpenCog
----------------
Perform the following steps at the shell prompt:
```
    cd to project root dir
    mkdir build
    cd build
    cmake ..
    make
```
Libraries will be built into subdirectories within build, mirroring
the structure of the source directory root.


Unit tests
----------
To build and run the unit tests, from the `./build` directory enter
(after building opencog as above):
```
    make test
```

Using OpenCog
-------------
OpenCog can be used in one of three ways, or a mixture of all three:
By using the GNU Guile scheme interface, by using Python, or by running
the cogserver.

Guile provides the easiest interface for creating atoms, loading them
into the AtomSpace, and performing various processing operations on
them.  For examples, see the `/examples/guile` and the
`/examples/pattern-matcher` directories.

Python is more familiar than scheme (guile) to most programmers, and
it offers another way of interfacing to the atomspace. See the
`/examples/python` directory for how to use python with OpenCog.

The cogserver provides a network server interface to OpenCog. It is
requires for running embodiment, some of the reasoning agents, and some
of the natural-language processing agents.

Running the server
------------------
The cogserver provides a network server interface to the various
components and agents.  After building everything, change directory
to your `opencog/build` folder and execute `opencog/cogserver/server/cogserver`.
Then, from another terminal, run `rlwrap telnet localhost 17001`
The `help` command will list all of the other available commands.
Notable among these are the commands to attach to a (Postgres) database,
and networked scheme and python interfaces (i.e. scheme and python
shells that are usable over the network, if you are logged in remotely
to the cogserver).

The operation of the server can be altered by means of a config file.
This config file is in `lib/opencog.conf`. To make use of it, say
`cogserver -c <config-filename>` when starting the server.


CMake notes
-----------
Some useful CMake's web sites/pages:

 - http://www.cmake.org (main page)
 - http://www.cmake.org/Wiki/CMake_Useful_Variables
 - http://www.cmake.org/Wiki/CMake_Useful_Variables/Get_Variables_From_CMake_Dashboards
 - http://www.cmake.org/Wiki/CMakeMacroAddCxxTest
 - http://www.cmake.org/Wiki/CMake_HowToFindInstalledSoftware


The main CMakeLists.txt currently sets -DNDEBUG. This disables Boost
matrix/vector debugging code and safety checks, with the benefit of
making it much faster. Boost sparse matrixes and (dense) vectors are
currently used by ECAN's ImportanceDiffusionAgent. If you use Boost
ublas in other code, it may be a good idea to at least temporarily
unset NDEBUG. Also if the Boost assert.h is used it will be necessary
to unset NDEBUG. Boost ublas is intended to respond to a specific
BOOST_UBLAS_NDEBUG, however this is not available as of the current
Ubuntu standard version (1.34).

-Wno-deprecated is currently enabled by default to avoid a number of
warnings regarding hash_map being deprecated (because the alternative
is still experimental!)

OpenCog
=======

OpenCog is a framework for developing AI systems, especially appropriate
for integrative multi-algorithm systems, and artificial general intelligence
systems.  Though much work remains to be done, it currently contains a
functional core framework, and a number of cognitive agents at varying levels
of completion, some already displaying interesting and useful functionalities
alone and in combination.

The main project site is at http://opencog.org

An interactive tutorial for getting started is available at:
https://github.com/opencog/opencog/blob/master/TUTORIAL.md

Prerequisites
-------------
To build and run OpenCog, the packages listed below are required. With a
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

###### guile
> Embedded scheme interpreter (version 2.0.0 or newer is required)
> http://www.gnu.org/software/guile/guile.html | guile-2.0-dev

###### libgsl
> The GNU Scientific Library
> http://www.gnu.org/software/gsl/ | libgsl0-dev

Optional Prerequisites
----------------------
The following packages are optional. If they are not installed, some
optional parts of OpenCog will not be built.  The CMake command, during
the build, will be more precise as to which parts will not be built.

###### curl
> cURL groks URLs
> Used by opencog/ubigraph
> http://curl.haxx.se/ | libcurl4-gnutls-dev

###### expat
> an XML parsing library
> Used by Embodiment subsystem
> http://expat.sourceforge.net/ | http://www.jclark.com/xml/expat.html (version 1.2) | libexpat1-dev

###### HyperTable
> Distributed storage
> http://hypertable.org
> This requires SIGAR as well

###### Link Grammar
> Natural Language Parser for English, Russian, other languages.
> Required for experimental Viterbi parser.
> http://www.abisource.com/projects/link-grammar/

###### MPI
> Message Passing Interface
> Required for compute-cluster version of MOSES
> Use either MPICHV2 or OpenMPI

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

###### unixODBC
> Generic SQL Database client access libraries
> Required for the distributed-processing atomspace.
> http://www.unixodbc.org/ | unixodbc-dev

###### xercesc
> Apache Xerces-C++ XML Parser
> Required for embodiment
> http://xerces.apache.org/xerces-c/ | libxerces-c2-dev

###### xmlrpc
> XML-RPC support
> Required by opencog/ubigraph
> http://www.xmlrpc.com | libxmlrpc-c-dev

###### ZeroMQ (version 3.2.4 or higher)
> Asynchronous messaging library
> http://zeromq.org/intro:get-the-software | libzmq3-dev

###### Threading Building Blocks
> C++ template library for parallel programming
> https://www.threadingbuildingblocks.org/download | libtbb-dev

Building OpenCog
----------------
Peform the following steps at the shell prompt:

    cd to project root dir
    mkdir build
    cd build
    cmake -DCMAKE_BUILD_TYPE=Release ..
    make

Libraries will be built into subdirectories within build, mirroring the
structure of the source directory root. The flag -DCMAKE_BUILD_TYPE=Release
results in binaries that are optimized for for performance; ommitting
this flag will result in faster builds, but slower executables.


Unit tests
----------
To build and run the unit tests, from the ./build directory enter (after
building opencog as above):

    make test


Running the server
------------------
The cogserver provides a simple server interface to the reasoning
system.

See CommandRequestProcessor.cc as an example control interface to
the server.  This command processor understands 3 simple commands:
load <xml file name>, ls and shutdown. There is an example XML file
under tests/server/atomSpace.xml

To run a simple test, build everything, change directory to your opencog/build
folder and execute opencog/server/cogserver. Then, from another terminal,
run ```telnet localhost 17001``` Try loading the example XML file and ls
to see all the nodes and links.


Config file
-----------
The operation of the server can be altered by means of a config file.
This config file is in lib/opencog.conf. To make use of it, say
```cogserver -c <config-filename>``` when starting the server.


Scheme shell
------------
The cog server also includes a built-in scheme shell. The shell can be
started by typing ```scm``` after entering the opencog server shell. It can
be exited by placing a single . on a line by itself.  This shell allows
opencog atoms and truth values to be created, manipulated and destroyed
using a very simple but powerful interface.  Examples and documentation
for the available OpenCog commands can be found in src/guile/README.
See also the wiki for additional details.


Modifying the list of basic types
---------------------------------
See the example under ./examples/atomtypes


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

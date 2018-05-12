OpenCog
=======

Build status:
[![Build Status](http://61.92.69.39:8080/buildStatus/icon?job=ci-opencog-master)](http://61.92.69.39:8080/job/ci-opencog-master)

OpenCog is a framework for developing AI systems, especially appropriate
for integrative multi-algorithm systems, and artificial general intelligence
systems.  Though much work remains to be done, it currently contains a
functional core framework, and a number of cognitive agents at varying levels
of completion, some already displaying interesting and useful functionalities
alone and in combination.

The main project site is at http://opencog.org

Overview
--------
OpenCog consists of multiple components. At its core is a (hyper-)graph
database, the [AtomSpace](http://github.com/opencog/atomspace), which is
used for representing knowledge and algorithms, providing a surface on
which learning and reasoning algorithms are implemented. The AtomSpace
consists of an in-RAM database, a "query language" aka "pattern matcher",
a (ProLog-like) rule system, including forward and backward chainers,
and an evaluator for the internal "programming langauge", Atomese. This
language is not really meant to be used by humans (although, defacto,
it is) but rather, it is a language for representing knowledge and
algorithms, on which (automated) reasoning and learning can be performed.
The AtomSpace also provides Scheme (guile) and Python bindings. The
AtomSpace is maintained in a separate git repo:
http://github.com/opencog/atomspace

This git repository contains assorted projects that are central to the
OpenCog project, but are not yet mature or stable, and are subject to
active development and experimentation. These include:
* An assortment of natural language processing subsystems, including:
-- Natural language generation (for expressiong thoughts as sentences).
-- Natural language input (for reading and hearing).
-- Assorted chatbots, some of which are embodied.
* PLN, a probabilistic reasoning and inference system.
* Attention Allocation, for managing combinatoric explosion during
  reasoning and language generation.
* Space-time servers, for managing spatial and time data (grounding
  common-sense natural language concepts such as "next-to", "nearby",
  and "soon".)
* An embodiment subsystem, attaching language to visual and auditory
  senses.  This is primarily located in the
  [ROS Behavior Scripting](https://github.com/opencog/ros-behavior-scripting)
  repository.
* OpenPsi, a model of psychological states. Its currently a mashup of
  two unrelated ideas: a generic rule-class selection and plannning
  system, and a model of human psychological states. An open to-do item
  is to untangle these two.
* An unsupervised learning system or "pattern miner", for extracting
  "surprising" patterns.
* A supervised learning system, MOSES, for extracting patterns from
  tabular data. This is located in a seprate repository,
  [MOSES](https://github.com/opencog/moses).
* The CogServer, a network server providing shell access and a REST API.
* Several (obsolete!?) data visualization subsystems.

With the exception of MOSES and the CogServer, all of the above are in
active development, are half-baked, poorly documented, mis-designed,
subject to experimentation, and generally in need of love an attention.
This is where experimentation and integration are taking place, and,
like any laboratory, things are a bit fluid and chaotic.


Building and Running
--------------------
For platform dependent instruction on dependencies and building the
code, as well as other options for setting up development environments,
more details are found on the [Building Opencog
wiki](http://wiki.opencog.org/wikihome/index.php/Building_OpenCog).

There is no single "demo" or system that can be "run"; rather, the
various subsystems can be run individually, or together. The single
most-fully-integrated, complete demo would be the embodied [Hanson
Robotics](http://github.com/hansonrobotics) chat subsystem.  This
can be run *without* having an actual robot; a virtual Blender
animation may be used instead; a webcam and microphones are required
for sensory input. Portions of this system can be found in the `nlp`
directory, in this repo, as well as the
[ROS Behavior Scripting](https://github.com/opencog/ros-behavior-scripting)
repo. The full setup is located in the Hanson Robotics
[HEAD](https://github.com/hansonrobotics/HEAD) repo, and ready-to-run
Docker images can be found in the [OpenCog Docker
repo](https://github.com/opencog/docker).


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
> http://github.com/opencog/cogutil
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
> Required for natural language generation, and the chatbot.
> http://www.abisource.com/projects/link-grammar/

###### MOSES
> MOSES Machine Learning
> http://github.com/opencog/moses
> It uses exactly the same build proceedure as this package. Be sure
  to `sudo make install` at the end.

###### OctoMap
> 3D occupancy grid mapping library
> Required for the robot perception subsystem.
> `sudo apt-get install liboctomap-dev`

Obsolete Prerequisites
----------------------
The following packages are needed to build some of the old, obsolete
packages.

###### CppREST
> C++ HTTP RESTful interfaces
> Used by the Pattern miner for distributed processing (this will be
  replaced by gearman in future releases).
> `sudo apt-get install libcpprest-dev`

###### Threading Building Blocks
> C++ template library for parallel programming
> Used to implement the optional REST API. (TODO: the REST API should
  be refactored to not use TBB)
> `sudo apt-get install libtbb-dev`

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

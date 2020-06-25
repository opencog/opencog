OpenCog
=======

[![CircleCI](https://circleci.com/gh/opencog/opencog.svg?style=svg)](https://circleci.com/gh/opencog/opencog)

OpenCog is a framework for developing AI systems, especially appropriate
for integrative multi-algorithm systems, and artificial general intelligence
systems.  Though much work remains to be done, it currently contains a
functional core framework, and a number of cognitive agents at varying levels
of completion, some already displaying interesting and useful functionalities
alone and in combination.

The main project site is at https://opencog.org

Overview
--------
OpenCog consists of multiple components. These can be found in assorted
git repos under https://github.com/opencog

This git repository contains an assortment of natural language tools,
embodied chatbots, and control/action-selection mechanisms.
These include:

* Ghost, a Chatscript-compatible chatbot with additional capabilities
  for accepting visual sensory input, and for controlling robot
  movements.

* OpenPsi, a model of psychological states. Its currently a mashup of
  two unrelated ideas: a generic rule-class action-selection and planning
  system, and a model of human psychological states. An open to-do item
  is to untangle these two.

* An assortment of natural language processing subsystems, including:
  * Natural language generation (for expressing thoughts as sentences).
  * Natural language input (for reading and hearing).
  * Relex2logic, converting natural language to logic expressions.
  * Assorted chatbots, some of which are embodied.
  * A lojban tool.

Prerequisites
-------------
To build and run OpenCog, the packages listed below are required.
With a few exceptions, most Linux distributions will provide these
packages. Users of Ubuntu may use the dependency installer from the
`/opencog/octool` repository.  Users of any version of Linux may
use the Dockerfile to quickly build a container in which OpenCog will
be built and run.

###### cogutil
> Common OpenCog C++ utilities
> https://github.com/opencog/cogutil
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### atomspace
> OpenCog Atomspace database and reasoning engine
> https://github.com/opencog/atomspace
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### cogserver
> OpenCog CogServer Network Server.
> https://github.com/opencog/cogserver
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### attention
> OpenCog Attention Allocation subsystem.
> https://github.com/opencog/attention
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### URE
> OpenCog Unified Rule Engine
> https://github.com/opencog/ure
> Required for PLN
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### pln
> OpenCog Probabilistic Logic Networks
> https://github.com/opencog/pln
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### spacetime
> OpenCog Spacetime Server - locations of objects in space and time.
> https://github.com/opencog/spacetime
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### ros-behavior-scripting
> Visual and auditory senses, robot motor control
> https://github.com/opencog/ros-behavior-scripting
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### Link Grammar
> Natural Language Parser for English, Russian, other languages.
> Required for natural language generation, and the chatbot.
> https://www.abisource.com/projects/link-grammar/


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

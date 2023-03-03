OpenCog
=======

[![CircleCI](https://circleci.com/gh/opencog/opencog.svg?style=svg)](https://circleci.com/gh/opencog/opencog)

***This repo is no longer maintained!***
Please use the following, instead:
* [opencog/atomspace](https://github.com/opencog/atomspace)
* [opencog/atomspace-cog](https://github.com/opencog/atomspace-cog)
* [opencog/atomspace-rocks](https://github.com/opencog/atomspace-rocks)
* [opencog/cogserver](https://github.com/opencog/cogserver)
* [opencog/lg-atomese](https://github.com/opencog/lg-atomese)
* [opencog/learn](https://github.com/opencog/learn)

***Obsolete!*** As of 2021, the most interesting and actively maintained
parts of this git repo have been split off into their own distinct git
repos.  What is left here is a mish-mash of unmatained stuff that is in
the process of bit-rotting. Some unit tests fail. Some unit tests
won't run. Some code won't compile. Perhaps there's some good stuff in
here. Perhaps it can be brought back to life and used for something or
other. However... for the most part, it is obsolete.

----------------------

This git repository contains the "OpenCog Framework", which has served
as a (scientific, technical) laboratory for researching, exploring and
learning how to integrate AI algorithms and systems into humanoid
robotic systems.  Most of the activity within this particular repo has
focused on integrating natural language chat, common-sense reasoning,
assorted learning algorithms, and motor control of humanoid robots.

A stated goal of the [OpenCog project](https://opencog.org) is to develop
artificial general intelligence (AGI) systems.  This is all and well;
however, what can be found here, in this particular repo, is very far
from that. The code here really is ... a laboratory for integrating
various types of AI systems.  As such, it is a compilation of several
decades of work by a large and varying collection of students,
researchers, professors and software engineers.  As a laboratory, it is
filled with all sorts of devices in varying states of working order,
from well-polished to mostly-broken.

See also:
* ROCCA - [Rational OpenCog Controlled Agent](https://github.com/opencog/rocca).
  This is a different assemblage of assorted OpenCog components,
  so that they operate within Minecraft, in the OpenAI Gym. The focus
  is on learning with the pattern miner, and reasoning with PLN.


Overview
--------
Most of the basic components used in OpenCog are distributed across
various git repos, (mostly) grouped under https://github.com/opencog

This git repository contains a crude natural language processing
pipeline, several embodied chatbots, and some control/action-selection
mechanisms.  These include:

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
  * A Lojban tool.

Prerequisites
-------------
To build and run the system here, the packages listed below are required.
Users of Ubuntu may use the dependency installer from the
[`/opencog/octool`](https://github.com/opencog/octool) repository.
Docker containers with OpenCog preconfigured can be found in the
[`opencog/docker`](https://github.com/opencog/docker) repo.

###### cogutil
> Common OpenCog C++ utilities.
> https://github.com/opencog/cogutil
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### atomspace
> OpenCog Atomspace, a sophisticated (hyper-)graph database.
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
> OpenCog Unified Rule Engine.
> https://github.com/opencog/ure
> Required for PLN
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### pln
> OpenCog Probabilistic Logic Networks reasoning system.
> https://github.com/opencog/pln
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### spacetime
> OpenCog Spacetime Server - locations of objects in space and time.
> https://github.com/opencog/spacetime
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### ros-behavior-scripting
> Visual and auditory senses, robot motor control.
> https://github.com/opencog/ros-behavior-scripting
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.

###### lg-atomese
> Natural Language Parser for English, Russian, other languages.
> Required for natural language generation, and the chatbot.
> https://github.com/opencog/lg-atomese
> It uses exactly the same build procedure as this package. Be sure
  to `sudo make install` at the end.


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

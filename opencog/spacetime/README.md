
SpaceTime Server
================

This code is part of a spatial-information processing system, described
here: https://wiki.opencog.org/w/SpaceServer

The actual implementation status of this code is unclear. I think it
works. I think parts of it have unit tests. I think parts of it might be
correctly designed and implemented. Parts of it might be incomplete or
incorrect or not needed. It might need changes to the API to become
fully useful.  The code here needs:

* An active maintainer!
* A full design review!

Atom Octomap API
----------------
This subsystem stores Atoms in a 4d spacetime coordinate map. The
current or past location of an atom can be queried, as well at the
set of atoms at a given location or time.

Scheme bindings are provided by the `(opencog pointmem)` module.

Documentation can be found here:
https://wiki.opencog.org/w/Visual_Perception_Buffer

Pre-requisites
-------------

In order to compile this code, this needs the OctoMap library:
```
sudo apt-get install liboctomap-dev
```

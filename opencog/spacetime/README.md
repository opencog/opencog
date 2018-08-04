=Deprecated!=

The system here is still operation, but it's use is deprecated!
A proper design, that will scale up for future users, and will
be easier to maintain, more powerful, and more flexibile, is
described in https://wiki.opencog.org/w/SpaceServer

***DO NOT USE THIS SYSTEM IN NEW CODE***

=Atom Octomap API=
This subsystem stores Atoms in a 4d spacetime coordinate map. The
current or past location of an atom can be queried, as well at the
set of atoms at a given location or time.

Scheme bindings are provided by the `(opencog pointmem)` module.

Documentation can be found here:
https://wiki.opencog.org/w/Visual_Perception_Buffer

==Pre-requistes==

In order to compile this code, this needs the OctoMap libary:
```
sudo apt-get install liboctomap-dev
```

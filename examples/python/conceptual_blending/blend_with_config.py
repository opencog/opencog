#! /usr/bin/env python
#
# blend_with_config.py
#
"""
Example usage of Conceptual Blending API.
Instantiates blender with a simple dataset stored in an AtomSpace
and learns a new concept.
For complete documentation on how to pass additional parameters to
blender, refer to the documentation at the following link:
https://github.com/opencog/opencog/tree/master/opencog/python/blending/blend-config-format.md
"""

__author__ = 'DongMin Kim'

from opencog.atomspace import AtomSpace
from opencog.utilities import initialize_opencog
from opencog.type_constructors import *

from blending.blend import ConceptualBlending

"""
Second Example:
- Blend with custom config.
- Give focus atom manually.
- Atoms that have STI value above 12 will be considered to blend.
- Force to start blend, and choose 2 nodes randomly.

Output dump:
--------Start second example--------
Source data:
[(ConceptNode "car" (av 17 0 0) (stv 1.000000 0.000000)) ; [2]
, (ConceptNode "man" (av 13 0 0) (stv 1.000000 0.000000)) ; [3]
, (ConceptNode "block" (av 5 0 0) (stv 1.000000 0.000000)) ; [4]
, (ConceptNode "build" (av 5 0 0) (stv 1.000000 0.000000)) ; [5]
]

Newly blended node:
[(ConceptNode "man-car" (av 0 0 0) (stv 1.000000 0.000000)) ; [389]
]
"""
print "--------Start second example--------"
a = AtomSpace()
initialize_opencog(a)

# Make custom concept network.
car = ConceptNode("car")
man = ConceptNode("man")
block = ConceptNode("block")
build = ConceptNode("build")
a.set_av(car.h, 17)
a.set_av(man.h, 13)
a.set_av(block.h, 5)
a.set_av(build.h, 5)
focus_atoms = [car, man, block, build]
print "Source data:\n" + \
      str(focus_atoms) + "\n"

# Make custom config.
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)

ExecutionLink(
    SchemaNode("BLEND:atoms-chooser"),
    ConceptNode("my-config"),
    ConceptNode("ChooseInSTIRange")
)

ExecutionLink(
    SchemaNode("BLEND:choose-sti-min"),
    ConceptNode("my-config"),
    ConceptNode("12")
)

ExecutionLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideRandom")
)

ExecutionLink(
    SchemaNode("BLEND:decide-result-atoms-count"),
    ConceptNode("my-config"),
    ConceptNode("2")
)

# Start Conceptual Blending.
result = ConceptualBlending(a).run(
    focus_atoms,
    ConceptNode("my-config")
)
print "Newly blended node: \n" + \
      str(result)

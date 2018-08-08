#! /usr/bin/env python
#
# blend_simple.py
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
First Example
- Blend with default config.

Output dump:
--------Start first example--------
Source data:
(ConceptNode "car") ; [2]
(ConceptNode "man") ; [3]

Newly blended node:
[(ConceptNode "car-man" (av 0 0 0) (stv 1.000000 0.000000)) ; [307]
]
"""
print "--------Start first example--------"

a = AtomSpace()
initialize_opencog(a)

# Make example concept network.
car = ConceptNode("car")
man = ConceptNode("man")
a.set_av(car.h, 17)
a.set_av(man.h, 13)
print "Source data:\n" + \
      str(car) + \
      str(man)

# Start Conceptual Blending.
result = ConceptualBlending(a).run()
print "Newly blended node: \n" + \
      str(result)

#! /usr/bin/env python
#
# blend_with_conflict_links.py
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
Third Example:
- Blend with custom config.
- Give focus atom manually.
- Force to start blend, and choose 2 nodes have highest STI value.
- Make 2^k available(viable) new blend atoms if there exists k conflicts,
  -> If two similarity links strength value have difference above 0.3 and 
     confidence value both above 0.7, blender thinks they conflict to
     each other.

Output dump:
--------Start third example--------
Source data:
[(ConceptNode "car" (av 19 0 0) (stv 1.000000 0.000000)) ; [2]
, (ConceptNode "man" (av 18 0 0) (stv 1.000000 0.000000)) ; [3]
, (ConceptNode "move" (av 2 0 0) (stv 1.000000 0.000000)) ; [5]
]

Newly blended nodes:
[(ConceptNode "car-man" (av 0 0 0) (stv 1.000000 0.000000)) ; [433]
, (ConceptNode "car-man-1" (av 0 0 0) (stv 1.000000 0.000000)) ; [536]
, (ConceptNode "car-man-2" (av 0 0 0) (stv 1.000000 0.000000)) ; [537]
, (ConceptNode "car-man-3" (av 0 0 0) (stv 1.000000 0.000000)) ; [538]
]

Detail information of selected conflict links:
-----
(ConceptNode "car-man") ; [433]

vehicle=> SimilarityLink (stv 0.100000 0.900000)
person=> SimilarityLink (stv 0.800000 0.900000)
-----
(ConceptNode "car-man-1") ; [536]

vehicle=> SimilarityLink (stv 0.100000 0.900000)
person=> SimilarityLink (stv 0.800000 0.900000)
-----
(ConceptNode "car-man-2") ; [537]

vehicle=> SimilarityLink (stv 0.100000 0.900000)
person=> SimilarityLink (stv 0.800000 0.900000)
-----
(ConceptNode "car-man-3") ; [538]

vehicle=> SimilarityLink (stv 0.100000 0.900000)
person=> SimilarityLink (stv 0.800000 0.900000)
"""
print "--------Start third example--------"
a = AtomSpace()
initialize_opencog(a)

# Make custom concept network.
"""
Make test nodes.
"""
# Nodes will be blended:
car = ConceptNode("car")
man = ConceptNode("man")

# A. Car is metal. (Not duplicated)
metal = ConceptNode("metal")

# B. Car moves, man moves. (Duplicated, not conflicted)
move = ConceptNode("move")

# C.1. Car is vehicle, man is not vehicle. (Duplicated and conflicted)
# C.2. Car is not person, man is person. (Duplicated and conflicted)
vehicle = ConceptNode("vehicle")
person = ConceptNode("person")

"""
Give some stimulates.
"""
a.set_av(car.h, 19)
a.set_av(man.h, 18)

a.set_av(metal.h, 1)
a.set_av(move.h, 2)

a.set_av(vehicle.h, 13)
a.set_av(person.h, 12)

"""
Make test links.
"""
# A. Not duplicated link.
# TODO: We should use an InheritanceLink rather than a MemberLink
# to express something in the some class. A case in below is just example
# to show how to blending algorithm works.
l1 = MemberLink(car, metal)
a.set_tv(l1.h, TruthValue(0.6, 0.8))

# B. Duplicated, not conflicted link.
# TODO: We should use an InheritanceLink rather than a SimilarityLink
# to express something has some property.
l2 = SimilarityLink(car, move)
l3 = SimilarityLink(man, move)
a.set_tv(l2.h, TruthValue(0.9, 0.8))
a.set_tv(l3.h, TruthValue(0.7, 0.9))

# C.1 Duplicated, conflicted link.
l4 = SimilarityLink(car, vehicle)
l5 = SimilarityLink(man, vehicle)
a.set_tv(l4.h, TruthValue(0.9, 0.8))
a.set_tv(l5.h, TruthValue(0.1, 0.9))

# C.2 Duplicated, conflicted link.
l6 = SimilarityLink(car, person)
l7 = SimilarityLink(man, person)
a.set_tv(l6.h, TruthValue(0.1, 0.8))
a.set_tv(l7.h, TruthValue(0.8, 0.9))

focus_atoms = [car, man, move]
print "Source data:\n" + \
      str(focus_atoms) + "\n"

# Make custom config.
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)
ExecutionLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideBestSTI")
)
ExecutionLink(
    SchemaNode("BLEND:link-connector"),
    ConceptNode("my-config"),
    ConceptNode("ConnectConflictAllViable")
)
ExecutionLink(
    SchemaNode("BLEND:connect-check-type"),
    ConceptNode("my-config"),
    ConceptNode("SimilarityLink")
)
ExecutionLink(
    SchemaNode("BLEND:connect-strength-diff-limit"),
    ConceptNode("my-config"),
    ConceptNode("0.3")
)
ExecutionLink(
    SchemaNode("BLEND:connect-confidence-above-limit"),
    ConceptNode("my-config"),
    ConceptNode("0.7")
)

# Start Conceptual Blending.
result = ConceptualBlending(a).run(
    focus_atoms,
    ConceptNode("my-config")
)
print "Newly blended nodes:"
print str(result) + "\n"

print "Detail information of selected conflict links:"
for node in result:
    print "-----"
    print node
    for sim_link in node.xincoming_by_type(types.SimilarityLink):
        for out_node in sim_link.out:
            if out_node.name == "vehicle" or out_node.name == 'person':
                print str(out_node.name) + "=> " + \
                      str(sim_link.type_name) + " " + \
                      str(sim_link.tv)

#! /usr/bin/env python
#
# blend.py
#
"""
Example usage of Conceptual Blending API.
Instantiates blender with a simple dataset stored in an AtomSpace
and learns a new concept.
For complete documentation on how to pass additional parameters to
blender, refer to the documentation at the following link:
https://github.com/opencog/opencog/tree/master/opencog/python/blending/doc/blend-config-format.md
"""

__author__ = 'DongMin Kim'

from opencog.type_constructors import *
from opencog.utilities import initialize_opencog
from blending.blend import ConceptualBlending

"""
Third Example:
- Blend with custom config.
- Give focus atom manually.
- Force to start blend, and choose 2 nodes have highest STI value.
- Make 2^k available(viable) new blend atoms if there exists k conflicts,
  -> If two similarity links strength value have difference above 0.3 and 
     confidence value both bave above 0.7, blender thinks they conflict to 
     each other.
"""
print "--------Start third example--------"
a = AtomSpace()
initialize_opencog(a)

# Make custom concept network.
vehicle = ConceptNode("vehicle")
human = ConceptNode("human")
thing = ConceptNode("thing")
metal = ConceptNode("metal")
a.set_av(vehicle.h, 11)
a.set_av(human.h, 25)
a.set_av(thing.h, 7)
a.set_av(metal.h, 20)

car = ConceptNode("car")
man = ConceptNode("man")
a.set_av(car.h, 50)
a.set_av(man.h, 64)

# A. Not duplicated link.
l1 = MemberLink(car, metal)
a.set_tv(l1.h, TruthValue(0.6, 0.8))

# B. Duplicated, not conflicted link.
l2 = SimilarityLink(car, thing)
l3 = SimilarityLink(man, thing)
a.set_tv(l2.h, TruthValue(0.9, 0.8))
a.set_tv(l3.h, TruthValue(0.7, 0.9))

# C.1 Duplicated, conflicted link.
l4 = SimilarityLink(car, vehicle)
l5 = SimilarityLink(man, vehicle)
a.set_tv(l4.h, TruthValue(0.9, 0.8))
a.set_tv(l5.h, TruthValue(0.1, 0.9))

# C.2 Duplicated, conflicted link.
l6 = SimilarityLink(car, human)
l7 = SimilarityLink(man, human)
a.set_tv(l6.h, TruthValue(0.1, 0.8))
a.set_tv(l7.h, TruthValue(0.8, 0.9))

focus_atoms = [car, man, thing]
print "Source data:\n" + \
      str(focus_atoms) + "\n"

# Make custom config.
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)
ListLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideBestSTI")
)
ListLink(
    SchemaNode("BLEND:link-connector"),
    ConceptNode("my-config"),
    ConceptNode("ConnectConflictAllViable")
)
ListLink(
    SchemaNode("BLEND:connect-check-type"),
    ConceptNode("my-config"),
    ConceptNode("SimilarityLink")
)
ListLink(
    SchemaNode("BLEND:connect-strength-diff-limit"),
    ConceptNode("my-config"),
    ConceptNode("0.3")
)
ListLink(
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
    for sim_link in a.xget_atoms_by_target_atom(types.SimilarityLink, node):
        for out_node in a.xget_outgoing(sim_link.h):
            if out_node.name == "vehicle" or out_node.name == 'human':
                print str(out_node.name) + "=> " + \
                      str(sim_link.type_name) + " " + \
                      str(sim_link.tv)

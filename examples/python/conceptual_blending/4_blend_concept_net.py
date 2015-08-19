#! /usr/bin/env python
#
# 4_blend_with_information_interaction_theory.py
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
from opencog.logger import log

from blending.blend import ConceptualBlending

"""
Fourth Example:
- Blend with custom config.
- Give focus atom manually.
- Choose one link set which has largest interaction information, and connect
  them to new blend.
"""
print "--------Start fourth example--------"

# Interaction Information algorithm takes very long time - about O(n*(2^n)) -
# if we not give any limit. If you want to check what is going now then you can
# enable below logger option.
log.use_stdout(True)
log.set_level("DEBUG")

a = AtomSpace()
initialize_opencog(a)

high_tv = TruthValue(0.7, 0.8)
low_tv = TruthValue(0.3, 0.8)

# 1. Define a super-class information
bat = ConceptNode("bat")
man = ConceptNode("man")

# TODO: Use correct Link(EatLink, ...) instead of using ListLink.
EvaluationLink(ListLink(bat, ConceptNode("steak"), low_tv))
InheritanceLink(bat, ConceptNode("two-legged"), low_tv)
InheritanceLink(bat, ConceptNode("fingered"), low_tv)
EvaluationLink(ListLink(bat, ConceptNode("flies"), high_tv))
InheritanceLink(bat, ConceptNode("winged"), high_tv)
EvaluationLink(ListLink(bat, ConceptNode("claws"), high_tv))
EvaluationLink(ListLink(bat, ConceptNode("sonar"), high_tv))
InheritanceLink(bat, ConceptNode("big"), low_tv)
InheritanceLink(bat, ConceptNode("popular"), low_tv)
EvaluationLink(ListLink(bat, ConceptNode("cave"), high_tv))
EvaluationLink(ListLink(bat, ConceptNode("gameboy"), low_tv))
EvaluationLink(ListLink(man, ConceptNode("nocturnal"), high_tv))

EvaluationLink(ListLink(man, ConceptNode("steak"), high_tv))
InheritanceLink(man, ConceptNode("two-legged"), high_tv)
InheritanceLink(man, ConceptNode("fingered"), high_tv)
EvaluationLink(ListLink(man, ConceptNode("flies"), low_tv))
InheritanceLink(man, ConceptNode("winged"), low_tv)
EvaluationLink(ListLink(man, ConceptNode("claws"), low_tv))
EvaluationLink(ListLink(man, ConceptNode("sonar"), low_tv))
InheritanceLink(man, ConceptNode("big"), low_tv)
InheritanceLink(man, ConceptNode("popular"), low_tv)
EvaluationLink(ListLink(man, ConceptNode("cave"), low_tv))
EvaluationLink(ListLink(man, ConceptNode("gameboy"), low_tv))
EvaluationLink(ListLink(man, ConceptNode("nocturnal"), low_tv))


# 2. Define sub-class information
funnel_eared = ConceptNode("funnel_eared")
megabats = ConceptNode("megabats")
vesper = ConceptNode("vesper")
night_stalker = ConceptNode("night_stalker")
zubat = ConceptNode("zubat")

scientist = ConceptNode("scientist")
police = ConceptNode("police")
programmer = ConceptNode("programmer")
ancients = ConceptNode("ancients")
vegetarian = ConceptNode("vegetarian")

InheritanceLink(funnel_eared, bat)
InheritanceLink(megabats, bat)
InheritanceLink(vesper, bat)
InheritanceLink(night_stalker, bat)
InheritanceLink(zubat, bat)

InheritanceLink(scientist, man)
InheritanceLink(police, man)
InheritanceLink(programmer, man)
InheritanceLink(ancients, man)
InheritanceLink(vegetarian, man)

# 3. Describe about the several bats.
EvaluationLink(ListLink(ConceptNode("funnel_eared"), ConceptNode("flies")))
InheritanceLink(ConceptNode("funnel_eared"), ConceptNode("winged"))
EvaluationLink(ListLink(ConceptNode("funnel_eared"), ConceptNode("claws")))
EvaluationLink(ListLink(ConceptNode("funnel_eared"), ConceptNode("sonar")))
EvaluationLink(ListLink(ConceptNode("funnel_eared"), ConceptNode("cave")))

EvaluationLink(ListLink(ConceptNode("megabats"), ConceptNode("flies")))
InheritanceLink(ConceptNode("megabats"), ConceptNode("winged"))
EvaluationLink(ListLink(ConceptNode("megabats"), ConceptNode("claws")))
EvaluationLink(ListLink(ConceptNode("megabats"), ConceptNode("sonar")))
InheritanceLink(ConceptNode("megabats"), ConceptNode("big"))
EvaluationLink(ListLink(ConceptNode("megabats"), ConceptNode("cave")))

EvaluationLink(ListLink(ConceptNode("vesper"), ConceptNode("flies")))
InheritanceLink(ConceptNode("vesper"), ConceptNode("winged"))
EvaluationLink(ListLink(ConceptNode("vesper"), ConceptNode("claws")))
EvaluationLink(ListLink(ConceptNode("vesper"), ConceptNode("sonar")))
InheritanceLink(ConceptNode("vesper"), ConceptNode("popular"))
EvaluationLink(ListLink(ConceptNode("vesper"), ConceptNode("cave")))

EvaluationLink(ListLink(ConceptNode("night_stalker"), ConceptNode("steak")))
EvaluationLink(ListLink(ConceptNode("night_stalker"), ConceptNode("flies")))
InheritanceLink(ConceptNode("night_stalker"), ConceptNode("winged"))
EvaluationLink(ListLink(ConceptNode("night_stalker"), ConceptNode("claws")))
EvaluationLink(ListLink(ConceptNode("night_stalker"), ConceptNode("sonar")))
EvaluationLink(ListLink(ConceptNode("night_stalker"), ConceptNode("cave")))

EvaluationLink(ListLink(ConceptNode("zubat"), ConceptNode("steak")))
InheritanceLink(ConceptNode("zubat"), ConceptNode("winged"))
EvaluationLink(ListLink(ConceptNode("zubat"), ConceptNode("claws")))
EvaluationLink(ListLink(ConceptNode("zubat"), ConceptNode("gameboy")))

# 4. Describe about the several men.
EvaluationLink(ListLink(ConceptNode("scientist"), ConceptNode("steak")))
InheritanceLink(ConceptNode("scientist"), ConceptNode("two-legged"))
InheritanceLink(ConceptNode("scientist"), ConceptNode("fingered"))

EvaluationLink(ListLink(ConceptNode("police"), ConceptNode("steak")))
InheritanceLink(ConceptNode("police"), ConceptNode("two-legged"))
InheritanceLink(ConceptNode("police"), ConceptNode("fingered"))

EvaluationLink(ListLink(ConceptNode("programmer"), ConceptNode("steak")))
InheritanceLink(ConceptNode("programmer"), ConceptNode("two-legged"))
InheritanceLink(ConceptNode("programmer"), ConceptNode("fingered"))
EvaluationLink(ListLink(ConceptNode("programmer"), ConceptNode("nocturnal")))

InheritanceLink(ConceptNode("ancients"), ConceptNode("two-legged"))
InheritanceLink(ConceptNode("ancients"), ConceptNode("fingered"))
EvaluationLink(ListLink(ConceptNode("ancients"), ConceptNode("cave")))

InheritanceLink(ConceptNode("vegetarian"), ConceptNode("two-legged"))
InheritanceLink(ConceptNode("vegetarian"), ConceptNode("fingered"))

# 5. Make custom config.
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)
ExecutionLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideNull")
)
ExecutionLink(
    SchemaNode("BLEND:link-connector"),
    ConceptNode("my-config"),
    ConceptNode("ConnectConflictInteractionInformation")
)
ExecutionLink(
    SchemaNode("BLEND:connect-check-type"),
    ConceptNode("my-config"),
    ConceptNode("ListLink")
)

# Start Conceptual Blending.
result = ConceptualBlending(a).run(
    [
        a.add_node(types.ConceptNode, "bat"),
        a.add_node(types.ConceptNode, "man")
    ],
    ConceptNode("my-config")
)


print "Newly blended node:"
print str(result[0]) + "\n"

print "Links in new blended node:"
print str(filter(lambda x: x.t == types.ListLink, result[0].incoming))

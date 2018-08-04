#! /usr/bin/env python
#
# blend_with_information_interaction.py
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

- Typical 'bat'
  -> eat flies
  -> has claws
  -> has sonar
  -> live in cave
  -> subclass of 'winged'
  -> subclass of 'nocturnal'

- Typical 'man'
  -> eat steak
  -> subclass of 'two-legged'
  -> subclass of 'fingered'

- Subset of 'bat'
  -> funnel_eared megabat vesper
  -> night_stalker(Eats steak)
  -> Zubat(Pokemon)
- Subset of 'man'
  -> scientist police programmer
  -> ancients(Lives in cave)
  -> vegetarian(Don't eats steak)

Output dump:
--------Start fourth example--------
ConnectConflictInteractionInformation: Calculating probabilities (Total: 10)
ConnectConflictInteractionInformation:PROB:: 1/10 (10.0%)
ConnectConflictInteractionInformation:PROB:: 4/10 (40.0%)
ConnectConflictInteractionInformation:PROB:: 7/10 (70.0%)
ConnectConflictInteractionInformation: Calculating interaction information (Total: 512)
ConnectConflictInteractionInformation:II:: 1/512 (0.1953125%)
ConnectConflictInteractionInformation:II:: 129/512 (25.1953125%)
ConnectConflictInteractionInformation:II:: 257/512 (50.1953125%)
ConnectConflictInteractionInformation:II:: 385/512 (75.1953125%)
# Link set that has largest interaction information value:
sonar, nocturnal, winged, fingered, cave, steak, claws, flies, two-legged, : 1.11336374283
...
# For example, an entity that has links:
# -> (eat steak)
# -> (eat flies) AND (has sonar) AND (has claws) AND (winged) AND (fingered) AND (two-legged)
#
# is more surprising than the entity that has:
# -> (live_in cave)
# -> (eat flies) AND (has sonar) AND (has claws) AND (winged) AND (fingered) AND (two-legged)
#
sonar, winged, fingered, steak, claws, flies, two-legged, : 1.0826215744
sonar, winged, fingered, cave, claws, flies, two-legged, : 1.07536005974
...
nocturnal, winged, fingered, cave, steak, claws, flies, two-legged, : 1.06969738007
sonar, nocturnal, winged, fingered, cave, claws, flies, two-legged, : 1.06638371944
...

Newly blended node:
(ConceptNode "bat-man") ; [443]

Links in new blended node:
[(InheritanceLink (av 0 0 0) (stv 1.000000 0.000000)
  (ConceptNode "bat-man" (av 0 0 0) (stv 1.000000 0.000000)) ; [443]
  (ConceptNode "man" (av 0 0 0) (stv 1.000000 0.000000)) ; [3]
) ; [658]
...
, (EvaluationLink (av 0 0 0) (stv 0.700000 0.800000)
  (PredicateNode "eat" (av 0 0 0) (stv 1.000000 0.000000)) ; [4]
  (ConceptNode "bat-man" (av 0 0 0) (stv 1.000000 0.000000)) ; [443]
  (ConceptNode "steak" (av 0 0 0) (stv 1.000000 0.000000)) ; [5]
) ; [655]
]

"""
print "--------Start fourth example--------"

# Interaction Information algorithm takes very long time - about O(n*(2^n)) -
# if we not give any limit. If you want to check what is going now then you can
# enable below logger option.
# log.use_stdout(True)
# log.set_level("DEBUG")

a = AtomSpace()
initialize_opencog(a)

high_tv = TruthValue(0.7, 0.8)
low_tv = TruthValue(0.3, 0.8)

# 1. Define a super-class information
bat = ConceptNode("bat")
man = ConceptNode("man")

eat = PredicateNode("eat")
steak = ConceptNode("steak")
flies = ConceptNode("flies")

has = PredicateNode("has")
claws = ConceptNode("claws")
sonar = ConceptNode("sonar")

live_in = PredicateNode("live_in")
cave = ConceptNode("cave")
pokeball = ConceptNode("pokeball")

two_legged = ConceptNode("two-legged")
fingered = ConceptNode("fingered")
winged = ConceptNode("winged")
huge_size = ConceptNode("huge_size")
popular = ConceptNode("popular")
nocturnal = ConceptNode("nocturnal")

# Bat
EvaluationLink(eat, bat, flies, high_tv)
EvaluationLink(eat, bat, steak, low_tv)

EvaluationLink(has, bat, claws, high_tv)
EvaluationLink(has, bat, sonar, high_tv)

EvaluationLink(live_in, bat, cave, high_tv)
EvaluationLink(live_in, bat, pokeball, low_tv)

InheritanceLink(bat, winged, high_tv)
InheritanceLink(bat, nocturnal, high_tv)
InheritanceLink(bat, two_legged, low_tv)
InheritanceLink(bat, fingered, low_tv)
InheritanceLink(bat, huge_size, low_tv)
InheritanceLink(bat, popular, low_tv)

# Man
EvaluationLink(eat, man, flies, low_tv)
EvaluationLink(eat, man, steak, high_tv)

EvaluationLink(has, man, claws, low_tv)
EvaluationLink(has, man, sonar, low_tv)

EvaluationLink(live_in, man, cave, low_tv)
EvaluationLink(live_in, man, pokeball, low_tv)

InheritanceLink(man, winged, low_tv)
InheritanceLink(man, nocturnal, low_tv)
InheritanceLink(man, two_legged, high_tv)
InheritanceLink(man, fingered, high_tv)
InheritanceLink(man, huge_size, low_tv)
InheritanceLink(man, popular, low_tv)


# 2. Define sub-class information
funnel_eared = ConceptNode("funnel_eared")
megabat = ConceptNode("megabat")
vesper = ConceptNode("vesper")
night_stalker = ConceptNode("night_stalker")
zubat = ConceptNode("zubat")

scientist = ConceptNode("scientist")
police = ConceptNode("police")
programmer = ConceptNode("programmer")
ancients = ConceptNode("ancients")
vegetarian = ConceptNode("vegetarian")

InheritanceLink(funnel_eared, bat)
InheritanceLink(megabat, bat)
InheritanceLink(vesper, bat)
InheritanceLink(night_stalker, bat)
InheritanceLink(zubat, bat)

InheritanceLink(scientist, man)
InheritanceLink(police, man)
InheritanceLink(programmer, man)
InheritanceLink(ancients, man)
InheritanceLink(vegetarian, man)


# 3. Describe about the several bats.
EvaluationLink(eat, funnel_eared, flies, high_tv)
EvaluationLink(has, funnel_eared, claws, high_tv)
EvaluationLink(has, funnel_eared, sonar, high_tv)
EvaluationLink(live_in, funnel_eared, cave, high_tv)
InheritanceLink(funnel_eared, winged, high_tv)
InheritanceLink(funnel_eared, nocturnal, high_tv)

EvaluationLink(eat, megabat, flies, high_tv)
EvaluationLink(has, megabat, claws, high_tv)
EvaluationLink(has, megabat, sonar, high_tv)
EvaluationLink(live_in, megabat, cave, high_tv)
InheritanceLink(megabat, winged, high_tv)
InheritanceLink(megabat, nocturnal, high_tv)
InheritanceLink(megabat, huge_size, high_tv)

EvaluationLink(eat, vesper, flies, high_tv)
EvaluationLink(has, vesper, claws, high_tv)
EvaluationLink(has, vesper, sonar, high_tv)
EvaluationLink(live_in, vesper, cave, high_tv)
InheritanceLink(vesper, winged, high_tv)
InheritanceLink(vesper, nocturnal, high_tv)
InheritanceLink(vesper, popular, high_tv)

# Night Stalker eats meat too.
EvaluationLink(eat, night_stalker, flies, high_tv)
EvaluationLink(eat, night_stalker, steak, high_tv)
EvaluationLink(has, night_stalker, claws, high_tv)
EvaluationLink(has, night_stalker, sonar, high_tv)
EvaluationLink(live_in, night_stalker, cave, high_tv)
InheritanceLink(night_stalker, winged, high_tv)
InheritanceLink(night_stalker, nocturnal, high_tv)

# The Zubat(Pokemon) lives in pokeball.
EvaluationLink(eat, zubat, steak, high_tv)
EvaluationLink(has, zubat, claws, high_tv)
EvaluationLink(has, zubat, sonar, high_tv)
EvaluationLink(live_in, zubat, pokeball, high_tv)
InheritanceLink(zubat, winged, high_tv)
InheritanceLink(zubat, nocturnal, high_tv)
InheritanceLink(zubat, popular, high_tv)


# 4. Describe about the several men.
EvaluationLink(eat, scientist, steak, high_tv)
InheritanceLink(scientist, two_legged, high_tv)
InheritanceLink(scientist, fingered, high_tv)

EvaluationLink(eat, police, steak, high_tv)
InheritanceLink(police, two_legged, high_tv)
InheritanceLink(police, fingered, high_tv)

EvaluationLink(eat, programmer, steak, high_tv)
InheritanceLink(programmer, two_legged, high_tv)
InheritanceLink(programmer, fingered, high_tv)
InheritanceLink(programmer, nocturnal, high_tv)

EvaluationLink(live_in, ancients, cave, high_tv)
InheritanceLink(ancients, two_legged, high_tv)
InheritanceLink(ancients, fingered, high_tv)

InheritanceLink(vegetarian, two_legged, high_tv)
InheritanceLink(vegetarian, fingered, high_tv)

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
    ConceptNode("Link")
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
print result[0].incoming

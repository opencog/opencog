from opencog.type_constructors import *
from opencog.utilities import initialize_opencog

from opencog_b.python.blending.blend import ConceptualBlending

__author__ = 'DongMin Kim'

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
# Make custom concept network end.

# Make custom config.
InheritanceLink(
    ConceptNode("awesome-config"),
    ConceptNode("BLEND")
)

ListLink(
    SchemaNode("BLEND:atoms-chooser"),
    ConceptNode("awesome-config"),
    ConceptNode("ChooseInSTIRange")
)

ListLink(
    SchemaNode("BLEND:choose-sti-min"),
    ConceptNode("awesome-config"),
    ConceptNode("12")
)

ListLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("awesome-config"),
    ConceptNode("DecideRandom")
)

ListLink(
    SchemaNode("BLEND:decide-result-atoms-count"),
    ConceptNode("awesome-config"),
    ConceptNode("2")
)
# Make custom config end.

# Start Conceptual Blending.
result = ConceptualBlending(a).run(
    focus_atoms,
    ConceptNode("awesome-config")
)
print result

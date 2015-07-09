from blending.util.blending_config import BlendConfig
from opencog.atomspace import AtomSpace
from opencog.type_constructors import *
from opencog.utilities import initialize_opencog
from opencog.logger import log

from blending.blend import ConceptualBlending

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

# Make custom config.
InheritanceLink(
    ConceptNode("my-config"),
    ConceptNode("BLEND")
)

ListLink(
    SchemaNode("BLEND:atoms-chooser"),
    ConceptNode("my-config"),
    ConceptNode("ChooseAll")
)

ListLink(
    SchemaNode("BLEND:blending-decider"),
    ConceptNode("my-config"),
    ConceptNode("DecideBestSTI")
)

ListLink(
    SchemaNode("BLEND:decide-result-atoms-count"),
    ConceptNode("my-config"),
    ConceptNode("2")
)

# Start Conceptual Blending.
result = ConceptualBlending(a).run(
    a.get_atoms_by_type(types.Atom),
    ConceptNode("my-config")
)

print "Newly blended node: \n" + \
      str(result)

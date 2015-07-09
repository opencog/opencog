from opencog.atomspace import AtomSpace, TruthValue
from opencog.type_constructors import types
from opencog.utilities import initialize_opencog
from opencog.logger import log

from blending.blend import ConceptualBlending

__author__ = 'DongMin Kim'

log.use_stdout()
log.set_level("WARN")

a = AtomSpace()
initialize_opencog(a)

# Make custom concept network.
"""
Make test nodes.
"""
# Nodes will be blended:
car = a.add_node(types.ConceptNode, "car")
man = a.add_node(types.ConceptNode, "man")
vehicle = a.add_node(types.ConceptNode, "vehicle")

a.set_av(car.h, 19)
a.set_av(man.h, 18)

"""
Make test links.
"""
l4 = a.add_link(types.SimilarityLink, [car, vehicle])
l5 = a.add_link(types.SimilarityLink, [man, vehicle])
a.set_tv(l4.h, TruthValue(0.9, 0.8))
a.set_tv(l5.h, TruthValue(0.1, 0.9))

result = ConceptualBlending(a).run()
print "Newly blended node: \n" + \
      str(result)

from opencog.type_constructors import *

from networks.network_loader import NetworkLoader
from opencog.utilities import initialize_opencog
from opencog_b.python.blending.blend import ConceptualBlending

__author__ = 'DongMin Kim'

a = AtomSpace()
initialize_opencog(a)

# Make example concept network.
NetworkLoader(a).make("PaulSallyNetwork")

# Start Conceptual Blending.
result = ConceptualBlending(a).run()
print result

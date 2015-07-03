from opencog.type_constructors import *
from opencog.utilities import initialize_opencog
from blending.blend import ConceptualBlending

__author__ = 'DongMin Kim'

a = AtomSpace()
initialize_opencog(a)

# Make example concept network.
car = ConceptNode("car")
man = ConceptNode("man")
a.set_av(car.h, 17)
a.set_av(man.h, 13)

# Start Conceptual Blending.
result = ConceptualBlending(a).run()
print result

from web.api.apimain import RESTAPI
from opencog.atomspace import AtomSpace, types
from opencog.utilities import initialize_opencog
from opencog.type_constructors import *

# Endpoint configuration
# To allow public access, set to 0.0.0.0; for local access, set to 127.0.0.1
IP_ADDRESS = '0.0.0.0'
PORT = 5000

atomspace = AtomSpace()
initialize_opencog(atomspace)

Link(
    ConceptNode("Test Concept"),
    ConceptNode("another one")
)

api = RESTAPI(atomspace)
api.run(host=IP_ADDRESS, port=PORT)

from opencog.atomspace import AtomSpace
from opencog.logger import log
from blending import ConceptualBlending
from opencog.utilities import initialize_opencog
from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import BlConfig, RESTAPILoader, BlLogger

__author__ = 'DongMin Kim'

a = AtomSpace()
initialize_opencog(a)

# Run RESTAPI server automatically to see my atomspace.
rest_api_loader = RESTAPILoader(a)
rest_api_loader.run('127.0.0.1', '5001')

# Start Conceptual Blending.
log.use_stdout()
blend = ConceptualBlending(a)

experiment_codes = ExperimentCodes(a)
experiment_codes.init_hook()

while 1:
    target_atoms = None
    config = None
    blend.run(target_atoms, config)

    BlLogger().log("Input n to stop, or continue.")
    is_stop = raw_input()

    if is_stop == 'n' or is_stop == 'N':
        break

experiment_codes.final_hook()

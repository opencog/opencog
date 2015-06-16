from examples.python.conceptual_blending.networks.network_finder import \
    NetworkLoader
from opencog.utilities import initialize_opencog
from opencog_b.python.conceptual_blending.conceptual_blending import \
    ConceptualBlending
from opencog_b.python.conceptual_blending.util.general_util import *

__author__ = 'DongMin Kim'

a = AtomSpace()
initialize_opencog(a)

# Run RESTAPI server automatically to see my atomspace.
rest_api_loader = RESTAPILoader(a)
rest_api_loader.run('127.0.0.1', '5001')


def test_make_focus_atoms(a):
    ret = None

    """
    ret = [
        a.add_node(types.ConceptNode, "car"),
        a.add_node(types.ConceptNode, "man")
    ]
    """

    NetworkLoader(a).make("PaulSallyNetwork")
    # NetworkLoader(a).make("DebateWithKantNetwork")

    return ret


def test_make_custom_config(a):
    awesome_config = a.add_node(types.ConceptNode, "awesome-config")

    a.add_link(
        types.InheritanceLink,
        [awesome_config, a.add_node(types.ConceptNode, "BLEND")]
    )

    a.add_link(
        types.ListLink,
        [
            a.add_node(types.SchemaNode, "BLEND:execute-mode"),
            awesome_config,
            a.add_node(types.ConceptNode, "Debug")
        ]
    )
    a.add_link(
        types.ListLink,
        [
            a.add_node(types.SchemaNode, "BLEND:make-atom-prefix"),
            awesome_config,
            a.add_node(types.ConceptNode, "(")
        ]
    )
    a.add_link(
        types.ListLink,
        [
            a.add_node(types.SchemaNode, "BLEND:make-atom-postfix"),
            awesome_config,
            a.add_node(types.ConceptNode, ")")
        ]
    )
    return awesome_config


# Start Conceptual Blending.
# log.use_stdout()
blend = ConceptualBlending(a)
# experiment_codes = ExperimentCodes(a)
# experiment_codes.init_hook()
focus_atoms = test_make_focus_atoms(a)
config_base = test_make_custom_config(a)
result = blend.run(focus_atoms, config_base)
"""
result = blend.run()
"""
BlLogger().log(result)

# experiment_codes.final_hook()
is_stop = raw_input("Enter to exit.")

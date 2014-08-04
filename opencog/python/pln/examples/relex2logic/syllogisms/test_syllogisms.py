__author__ = 'sebastian'

# Socrates example is a categorical syllogism --> move in this directory?

"""
For testing syllogisms
"""

from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from opencog.cogserver import MindAgent
from opencog.atomspace import types
from pln.chainers import Chainer
from pln.rules import *

__author__ = 'Sebastian Ruder'

rules = {"ModusPonensRule": ModusPonensRule}

link_types = {"ImplicationLink": types.ImplicationLink,
              "InheritanceLink": types.InheritanceLink}


def create_chainer(atomspace, rule_concept_nodes):
    """
    :param atomspace: the atomspace to be used by the chainer
    :param concept_nodes: list of ConceptNodes where the name is the name of a
    rule optionally amended by a link_type separated with ":", e.g.
    [ConceptNode "ModusPonensRule:ImplicationLink"]
    :return: the chainer that has been created
    """
    chainer = Chainer(atomspace,
                      stimulateAtoms=False,
                      allow_output_with_variables=True,
                      delete_temporary_variables=True)
    for rule_concept_node in rule_concept_nodes:
        rule_name, link_type = rule_concept_node.name.split(":")
        chainer.add_rule(rules[rule_name](chainer, link_types[link_type]))
    return chainer


def get_predicate_arguments(atomspace, predicate_name):
    """
    Finds the unique EvaluationLink for a predicate and returns its list
    of arguments.
    (Adapted from Alex's PLNUnitTester class)
    :param atomspace: the atomspace where the predicate should be looked for
    :param predicate_name: the name of the predicate
    :return: a list of the predicate arguments
    """
    chainer = Chainer(atomspace)
    var = chainer.new_variable()
    predicate = chainer.atomspace.add_node(types.PredicateNode, predicate_name)
    template = chainer.link(types.EvaluationLink,
                            [predicate, var])

    queries = chainer.lookup_atoms(template, "")
    # The template needs to be removed from the results
    queries.remove(template)
    if len(queries) != 1:
        raise ValueError("Predicate " + predicate_name +
                         " must have 1 EvaluationLink")
    return queries[0].out[1].out


def set_up_atomspace():
    """
    Initializes a new atomspace, loads the core types and utitities.
    :return: an atomspace
    """
    coreTypes = "opencog/atomspace/core_types.scm"
    utilities = "opencog/scm/utilities.scm"
    newly_created_atomspace = AtomSpace()
    __init__(newly_created_atomspace)
    for item in [coreTypes, utilities]:
        load_scm(newly_created_atomspace, item)
    return newly_created_atomspace


def transfer_atom(new_atomspace, atom):
    """
    Transfers (or rather copies) an atom from one atomspace to another under
    the assumption that the atomspaces have the same list of atom types.

    returns the equivalent of atom in new_atomspace. creates it if
    necessary, including the outgoing set of links.
    :param new_atomspace: the atomspace where the atoms should be copied to
    :param atom: the atom to be copied
    :return: the added node or link
    """
    if atom.is_node():
        return new_atomspace.add_node(atom.type, atom.name, tv=atom.tv)
    else:
        outgoing = [transfer_atom(new_atomspace, out) for out in atom.out]
        return new_atomspace.add_link(atom.type, outgoing, tv=atom.tv)

# Parameters
num_steps = 100
print_starting_contents = True

# Syllogism files
syllogisms = [
    # "abductive-syllogism.scm",
    # "conditional-syllogism.scm",
    # "disjunctive-syllogism.scm",
    # "easy-syllogisms.scm",
    # "hard-syllogisms.scm",
    # "medium-syllogisms.scm",
    # "syllogism-boat.scm",
    "syllogism-canadian.scm",
    # "wrong-can-be-right.scm"
    ]

syllogism_path = "opencog/python/pln/examples/relex2logic/syllogisms/"

for syllogism in syllogisms:
    # sets up atomspace where inputs, rules, and outputs can be retrieved
    configuration_space = set_up_atomspace()
    load_scm(configuration_space, syllogism_path + syllogism)

    # sets up atomspace that is used for the actual inference
    inference_space = set_up_atomspace()
    for atom in get_predicate_arguments(configuration_space, "inputs"):
        transfer_atom(inference_space, atom)

    if print_starting_contents:
        print('AtomSpace starting contents:')
        inference_space.print_list()

    syllogism_chainer = create_chainer(
        inference_space, get_predicate_arguments(configuration_space, "rules"))

    outputs_produced = 0

    for i in range(0, num_steps):
        result = syllogism_chainer.forward_step()

        if result is not None:
            (rule, input, output) = result
            outputs_produced += 1

            print("\n----- [Output # {0}] -----".format(outputs_produced))
            for j in output:
                print("-- Output:\n{0}".format(j))
            print("-- using production rule: {0}".format(rule.name))
            print("\n-- based on this input:\n{0}".format(input))

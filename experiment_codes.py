__author__ = 'kdm'

import os.path;
import sys

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.bindlink import bindlink
from opencog.type_constructors \
    import ConceptNode, TypeNode, VariableNode,\
        UnorderedLink, MemberLink, InheritanceLink
from opencog.utilities import initialize_opencog
from opencog.logger import log
from opencog.scheme_wrapper \
    import load_scm, scheme_eval, scheme_eval_h, __init__

sys.path.append(os.path.expanduser("~/opencog/opencog/python"))
import web.api.restapi

# Paul & Sally example in the book 'The Way We Think'
# It is a simplex network example.
class PaulSallyExample:
    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []

    # space: part of whole knowledge(atoms) from database = atomspace
    # frame: some principle to extract & make space
    def _make_default_concept(self):
        # Dummy truth values.
        self.atom_tv = TruthValue(0.9, 0.8)
        self.link_tv = TruthValue(0.7, 0.6)

    # - Input Space 0
    # Start to make 'Family' network.
    # It has frame. It not has variables.
    def _make_input_space_0(self):
        # Make instance of space&frame concept.
        a_input_space_0 = ConceptNode("InputSpace0", self.atom_tv)
        a_family = ConceptNode("Family", self.atom_tv)

        # Make role concept.
        a_grand_father = ConceptNode("Grandfather", self.atom_tv)
        a_grand_mother = ConceptNode("Grandmother", self.atom_tv)
        a_father = ConceptNode("Father", self.atom_tv)
        a_mother = ConceptNode("Mother", self.atom_tv)
        a_son = ConceptNode("Son", self.atom_tv)
        a_daughter = ConceptNode("Daughter", self.atom_tv)

        # Link with family frame type.
        l_family_0 = MemberLink(a_grand_father, a_family, self.link_tv)
        l_family_1 = MemberLink(a_grand_mother, a_family, self.link_tv)
        l_family_2 = MemberLink(a_father, a_family, self.link_tv)
        l_family_3 = MemberLink(a_mother, a_family, self.link_tv)
        l_family_4 = MemberLink(a_son, a_family, self.link_tv)
        l_family_5 = MemberLink(a_daughter, a_family, self.link_tv)

        # Link with input space 0 type.
        l_input_space_0_0 = MemberLink(
            a_father, a_input_space_0, self.link_tv)
        l_input_space_0_1 = MemberLink(
            a_daughter, a_input_space_0, self.link_tv)

        """
        # Define new space&frame by linking.
        l_space_0 = InheritanceLink(
            a_input_space_0,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.link_tv)
        l_frame_0 = InheritanceLink(
            a_family,
            self.a.get_atoms_by_name(
                types.Atom, "Frame")[0],
            self.link_tv)
        """

    # - Input Space 1
    # Start to make 'actual human' network.
    # It not has frame. It has variables.
    def _make_input_space_1(self):
        # Make instance of space concept.
        a_input_space_1 = ConceptNode("InputSpace1", self.atom_tv)

        # Make variable concept.
        a_paul = VariableNode("Paul", TruthValue(0.9, 0.8))
        a_sally = VariableNode("Sally", TruthValue(0.9, 0.8))

        # Link with input space 1 type.
        l_input_space_1_0 = MemberLink(a_paul, a_input_space_1, self.link_tv)
        l_input_space_1_1 = MemberLink(a_sally, a_input_space_1, self.link_tv)

        """
        # Define new space by linking.
        l_space_1 = InheritanceLink(
            a_input_space_1,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.link_tv)
        """

    # - Generic Space
    def _make_generic_space(self):
        # Make instance of space concept.
        a_generic_space = ConceptNode("GenericSpace", self.atom_tv)

        # Make role concept.
        a_human = ConceptNode("Human", self.atom_tv)
        a_man = ConceptNode("Man", self.atom_tv)
        a_woman = ConceptNode("Woman", self.atom_tv)

        # Link with man type.
        l_man_0 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Grandfather")[0],
            a_man,
            self.link_tv)
        l_man_1 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Father")[0],
            a_man,
            self.link_tv)
        l_man_2 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Son")[0],
            a_man,
            self.link_tv)
        l_man_3 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Paul")[0],
            a_man,
            self.link_tv)

        # Link with woman type.
        l_woman_0 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Grandmother")[0],
            a_woman,
            self.link_tv)
        l_woman_1 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Mother")[0],
            a_woman,
            self.link_tv)
        l_woman_2 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Woman")[0],
            a_woman,
            self.link_tv)
        l_woman_3 = InheritanceLink(
            self.a.get_atoms_by_name(
                types.Atom, "Sally")[0],
            a_woman,
            self.link_tv)

        """
        # Link with human being type.
        l_human_0 = InheritanceLink(a_grand_father, a_human, self.link_tv)
        l_human_1 = InheritanceLink(a_grand_mother, a_human, self.link_tv)
        l_human_2 = InheritanceLink(a_father, a_human, self.link_tv)
        l_human_3 = InheritanceLink(a_mother, a_human, self.link_tv)
        l_human_4 = InheritanceLink(a_son, a_human, self.link_tv)
        l_human_5 = InheritanceLink(a_daughter, a_human, self.link_tv)
        """

        # TODO: deep inheritance or direct inheritance?
        # Link with human being type.
        l_human_0 = InheritanceLink(a_man, a_human, self.link_tv)
        l_human_1 = InheritanceLink(a_woman, a_human, self.link_tv)

        # Link with input space 0 type.
        l_generic_space_0 = MemberLink(
            a_man,
            a_generic_space,
            self.link_tv)
        l_generic_space_1 = MemberLink(
            a_woman,
            a_generic_space,
            self.link_tv)
        l_generic_space_1 = MemberLink(
            a_human,
            a_generic_space,
            self.link_tv)

        """
        # Define new space by linking.
        l_space_2 = InheritanceLink(
            a_generic_space,
            self.a.get_atoms_by_name(
                types.Atom, "Space"),
            self.link_tv)
        """

    # - Blended Space
    def _make_blended_space(self):
        # do nothing
        print ""

    def make(self):
        self._make_default_concept()
        self._make_input_space_0()
        self._make_input_space_1()
        self._make_generic_space()


# Make several test cases for debug.
class TestCaseMaker:
    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []
        self._make_default_concept()
        self.paul_sally_example = PaulSallyExample(self.a)

    def _make_default_concept(self):
        # Dummy truth values.
        self.atom_tv = TruthValue(1, 1)
        self.link_tv = TruthValue(1, 1)
        # - Base Concepts
        # Make space & frame concept.
        self.a_space = ConceptNode("Space", self.atom_tv)
        self.a_frame = ConceptNode("Frame", self.atom_tv)

    def make(self):
        self.paul_sally_example.make()
        # self.atom_list_for_debug.extend([a1, a2])
        # self.link_list_for_debug.extend([l1])


# Note: Divided to standalone class because I'll remove
class RESTAPILoader:
    def __init__(self, atomspace):
        self.a = atomspace
        self.restapi = web.api.restapi.Start()

    def run(self):
        self.restapi.run("", self.a)


# Class for dongmin's practice & experiment & test
class ToyClass:
    def __init__(self, atomspace):
        self.a = atomspace

    def foo(self):
        # patter matcher written by Cosmo Harrigan (bindlink.py)
        data = [os.path.expanduser
                ("~/atomspace/build/opencog/atomspace/core_types.scm"),
                os.path.expanduser
                ("~/atomspace/opencog/scm/*.scm")
                ]

        for item in data:
            load_scm(self.a, item)

        scheme_animals = \
            '''
            (InheritanceLink (ConceptNode "Frog") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Zebra") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Deer") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Spaceship") (ConceptNode "machine"))
            '''

        scheme_eval_h(self.a, scheme_animals)

        scheme_query = \
            '''
            (BindLink
                ;; The variable to be bound
                (VariableNode "$var")
                (ImplicationLink
                ;; The pattern to be searched for
                (InheritanceLink
                    (VariableNode "$var")
                    (ConceptNode "animal")
                )
                ;; The value to be returned.
                (VariableNode "$var")
                )
            )
            '''
        bind_link_handle = scheme_eval_h(self.a, scheme_query)

        # Run the above pattern and print the result
        result = bindlink(self.a, bind_link_handle)
        print "The result of pattern matching is:\n\n" + str(self.a[result])


class ExperimentCodes:
    def __init__(self, atomspace):
        self.a = atomspace
        self.test_case_maker = TestCaseMaker(self.a)
        self.toy_class = ToyClass(self.a)

    def execute(self):
        self.test_case_maker.make()
        # DEBUG: Start Toy method.
        # self.toy_class.foo()

        # DEBUG: Run RESTAPI server automatically to see my atomspace.
        rest_api_loader = RESTAPILoader(self.a)
        rest_api_loader.run()
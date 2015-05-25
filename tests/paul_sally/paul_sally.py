from tests.base_test_case import BaseTestCase
from util import blending_util
from util.blending_util import *

__author__ = 'DongMin Kim'

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import types, TruthValue
from opencog.type_constructors \
    import ConceptNode, VariableNode, \
    MemberLink, InheritanceLink

# Paul & Sally example in the book 'The Way We Think'
# It is a simplex network example.
class PaulSallyExample(BaseTestCase):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return 'PaulSallyExample'

    # - Input Space 0
    # Start to make 'Family' network.
    # It has frame. It not has variables.
    def __make_input_space_0(self):
        # Make instance of space&frame concept.
        a_input_space_0 = ConceptNode("InputSpace0", self.default_atom_tv)
        a_family = ConceptNode("Family", rand_tv())

        # Make role concept.
        a_grand_father = ConceptNode("Grandfather", rand_tv())
        a_grand_mother = ConceptNode("Grandmother", rand_tv())
        a_father = ConceptNode("Father", rand_tv())
        a_mother = ConceptNode("Mother", rand_tv())
        a_son = ConceptNode("Son", rand_tv())
        a_daughter = ConceptNode("Daughter", rand_tv())

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_grand_father, a_grand_mother,
             a_father, a_mother, a_son, a_daughter],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with family frame type.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_grand_father, a_grand_mother,
             a_father, a_mother, a_son, a_daughter],
            a_family
        )

        # Link with input space 0 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_father, a_daughter],
            a_input_space_0,
            self.default_link_tv
        )

        # Define new space&frame by linking.
        l_space_0 = InheritanceLink(
            a_input_space_0,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.default_link_tv)
        l_frame_0 = InheritanceLink(
            a_family,
            self.a.get_atoms_by_name(
                types.Atom, "Frame")[0],
            self.default_link_tv)

    # - Input Space 1
    # Start to make 'actual human' network.
    # It not has frame. It has variables.
    def __make_input_space_1(self):
        # Make instance of space concept.
        a_input_space_1 = ConceptNode("InputSpace1", self.default_atom_tv)

        # Make variable concept.
        a_paul = VariableNode("Paul", rand_tv())
        a_sally = VariableNode("Sally", rand_tv())

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_paul, a_sally],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with input space 1 type.
        l_input_space_1_0 = MemberLink(a_paul, a_input_space_1, self.default_link_tv)
        l_input_space_1_1 = MemberLink(a_sally, a_input_space_1, self.default_link_tv)

        # Define new space by linking.
        l_space_1 = InheritanceLink(
            a_input_space_1,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.default_link_tv)

    # - Generic Space
    def __make_generic_space(self):
        # Make instance of space concept.
        a_generic_space = ConceptNode("GenericSpace", self.default_atom_tv)

        # Make role concept.
        a_human = ConceptNode("Human", rand_tv())
        a_man = ConceptNode("Man", rand_tv())
        a_woman = ConceptNode("Woman", rand_tv())

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_human, a_man, a_woman],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with man type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a.get_atoms_by_name
                (types.Atom, "Grandfather")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Father")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Son")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Paul")[0]
            ],
            a_man
        )

        # Link with woman type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a.get_atoms_by_name
                (types.Atom, "Grandmother")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Mother")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Daughter")[0],
                self.a.get_atoms_by_name
                (types.Atom, "Sally")[0]
            ],
            a_woman
        )

        """
        # TODO: deep inheritance or direct inheritance?
        # Link with human being type.
        l_human_0 = InheritanceLink(a_grand_father, a_human, self.link_tv)
        l_human_1 = InheritanceLink(a_grand_mother, a_human, self.link_tv)
        l_human_2 = InheritanceLink(a_father, a_human, self.link_tv)
        l_human_3 = InheritanceLink(a_mother, a_human, self.link_tv)
        l_human_4 = InheritanceLink(a_son, a_human, self.link_tv)
        l_human_5 = InheritanceLink(a_daughter, a_human, self.link_tv)
        """

        # Link with human being type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [a_man, a_woman],
            a_human
        )

        # Link with input space 0 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [a_man, a_woman, a_human],
            a_generic_space,
            self.default_link_tv
        )

        # Define new space by linking.
        l_space_2 = InheritanceLink(
            a_generic_space,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.default_link_tv)

    # - Blended Space
    def __make_blended_space(self):
        # Make instance of space concept.
        a_blended_space = ConceptNode("BlendedSpace", self.default_atom_tv)

    def make(self):
        self.__make_input_space_0()
        self.__make_input_space_1()
        self.__make_generic_space()
        self.__make_blended_space()

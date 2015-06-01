from tests_b.base_test_case import BaseTestCase
from util_b.blending_util import *
from util_b.general_util import BlendingConfigLoader

__author__ = 'DongMin Kim'

from opencog.type_constructors import *


# Paul & Sally example in the book 'The Way We Think'
# It is a simplex network example.
class PaulSallyExample(BaseTestCase):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return 'PaulSallyExample'

    # Make all concept and link.
    def __make_atoms(self):
        #
        # Make nodes.
        #

        # Make default concepts.
        self.a_space = self.a.get_atoms_by_name(types.Atom, "Space", True)[0]
        self.a_frame = self.a.get_atoms_by_name(types.Atom, "Frame", True)[0]
        self.a_input_space_0 = ConceptNode("InputSpace0", self.default_atom_tv)
        self.a_input_space_1 = ConceptNode("InputSpace1", self.default_atom_tv)
        self.a_generic_space = ConceptNode("GenericSpace", self.default_atom_tv)
        self.a_blended_space = ConceptNode("BlendedSpace", self.default_atom_tv)

        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a_input_space_0,
                self.a_input_space_1,
                self.a_generic_space,
                self.a_blended_space
            ],
            self.a_space,
            self.default_link_tv
        )

        # Use in InputSpace0.
        self.a_family = ConceptNode("Family", rand_tv())
        self.a_grand_father = ConceptNode("Grandfather", rand_tv())
        self.a_grand_mother = ConceptNode("Grandmother", rand_tv())
        self.a_father = ConceptNode("Father", rand_tv())
        self.a_mother = ConceptNode("Mother", rand_tv())
        self.a_son = ConceptNode("Son", rand_tv())
        self.a_daughter = ConceptNode("Daughter", rand_tv())
        make_sti_all(
            self.a,
            [self.a_family,
             self.a_grand_father, self.a_grand_mother,
             self.a_father, self.a_mother,
             self.a_son, self.a_daughter],
            self.JUST_TARGET
        )

        # Use in InputSpace1.
        self.a_paul = VariableNode("Paul", rand_tv())
        self.a_sally = VariableNode("Sally", rand_tv())
        make_sti_all(
            self.a,
            [self.a_paul, self.a_sally],
            self.JUST_TARGET
        )

        # Use in Generic Space.
        self.a_human = ConceptNode("Human", rand_tv())
        self.a_man = ConceptNode("Man", rand_tv())
        self.a_woman = ConceptNode("Woman", rand_tv())
        make_sti_all(
            self.a,
            [self.a_human, self.a_human, self.a_woman],
            self.JUST_TARGET
        )

    # - Input Space 0
    # Start to make 'Family' network.
    # It has frame. It not has variables.
    def __make_input_space_0(self):
        # Make instance of frame concept.
        InheritanceLink(self.a_family, self.a_frame, self.default_link_tv)
        # Link with family frame type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_grand_father, self.a_grand_mother,
             self.a_father, self.a_mother,
             self.a_son, self.a_daughter],
            self.a_family
        )

        # Make role concept.
        make_sti_all(
            self.a,
            [self.a_father, self.a_daughter],
            self.IMPORTANT
        )
        # Link with input space 0 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_father, self.a_daughter],
            self.a_input_space_0,
            self.default_link_tv
        )

    # - Input Space 1
    # Start to make 'actual human' network.
    # It not has frame. It has variables.
    def __make_input_space_1(self):
        # Make variable concept.
        make_sti_all(
            self.a,
            [self.a_paul, self.a_sally],
            self.IMPORTANT
        )
        # Link with input space 1 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_paul, self.a_sally],
            self.a_input_space_1,
            self.default_link_tv
        )

    # - Generic Space
    def __make_generic_space(self):
        # Link with human being type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.a_man, self.a_woman],
            self.a_human
        )

        """
        # Link with human being type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a_grand_father, self.a_grand_mother,
                self.a_father, self.a_mother,
                self.a_son, self.a_daughter,
                self.a_paul, self.a_sally
            ],
            self.a_human
        )
        """

        # Link with man type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a_grand_father, self.a_father,
                self.a_son,
                self.a_paul
            ],
            self.a_man
        )

        # Link with woman type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.a_grand_mother, self.a_mother,
                self.a_daughter,
                self.a_sally
            ],
            self.a_woman
        )

        # Link with generic space type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_man, self.a_woman, self.a_human],
            self.a_generic_space
        )

    # - Blended Space
    def __make_blended_space(self):
        pass

    def __link_with_blend_target_for_debug(self):
        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_grand_father, self.a_grand_mother,
             self.a_father, self.a_mother,
             self.a_son, self.a_daughter],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_paul, self.a_sally],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with blend target.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.a_human, self.a_human, self.a_woman],
            self.a_blend_target,
            blend_target_link_tv
        )

    def make(self):
        self.__make_atoms()
        self.__make_input_space_0()
        self.__make_input_space_1()
        self.__make_generic_space()
        self.__make_blended_space()

        if BlendingConfigLoader().is_use_blend_target:
            self.__link_with_blend_target_for_debug()

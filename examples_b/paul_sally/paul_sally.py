from examples_b.base_example import BaseExample
from util_b.blending_util import *
from opencog.type_constructors import *

__author__ = 'DongMin Kim'


# Paul & Sally example in the book 'The Way We Think'
# It is a simplex network example.
class PaulSallyExample(BaseExample):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return self.__class__.__name__

    # Make all concept and link.
    def __make_atoms(self):
        #
        # Make nodes.
        #

        # Make default concepts.
        self.space = ConceptNode("Space", self.default_atom_tv)
        self.frame = ConceptNode("Frame", self.default_atom_tv)
        self.input_space_0 = ConceptNode("InputSpace0", self.default_atom_tv)
        self.input_space_1 = ConceptNode("InputSpace1", self.default_atom_tv)
        self.generic_space = ConceptNode("GenericSpace", self.default_atom_tv)
        self.blend_space = ConceptNode("BlendSpace", self.default_atom_tv)

        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.input_space_0,
                self.input_space_1,
                self.generic_space,
                self.blend_space
            ],
            self.space,
            self.default_link_tv
        )

        # Use in InputSpace0.
        self.family = ConceptNode("Family", rand_tv())
        self.grand_father = ConceptNode("Grandfather", rand_tv())
        self.grand_mother = ConceptNode("Grandmother", rand_tv())
        self.father = ConceptNode("Father", rand_tv())
        self.mother = ConceptNode("Mother", rand_tv())
        self.son = ConceptNode("Son", rand_tv())
        self.daughter = ConceptNode("Daughter", rand_tv())
        make_sti_all(
            [self.family,
             self.grand_father, self.grand_mother,
             self.father, self.mother,
             self.son, self.daughter],
            sti_value_dict['JUST_TARGET']
        )
        # Use in InputSpace1.
        self.paul = VariableNode("Paul", rand_tv())
        self.sally = VariableNode("Sally", rand_tv())
        make_sti_all(
            [self.paul, self.sally],
            sti_value_dict['JUST_TARGET']
        )

        # Use in Generic Space.
        self.human = ConceptNode("Human", rand_tv())
        self.man = ConceptNode("Man", rand_tv())
        self.woman = ConceptNode("Woman", rand_tv())
        make_sti_all(
            [self.human, self.human, self.woman],
            sti_value_dict['JUST_TARGET']
        )

    # - Input Space 0
    # Start to make 'Family' network.
    # It has frame. It not has variables.
    def __make_input_space_0(self):
        # Make instance of frame concept.
        InheritanceLink(self.family, self.frame, self.default_link_tv)
        # Link with family frame type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.grand_father, self.grand_mother,
             self.father, self.mother,
             self.son, self.daughter],
            self.family
        )

        # Make role concept.
        make_sti_all(
            [self.father, self.daughter],
            sti_value_dict['IMPORTANT']
        )
        # Link with input space 0 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.father, self.daughter],
            self.input_space_0,
            self.default_link_tv
        )

    # - Input Space 1
    # Start to make 'actual human' network.
    # It not has frame. It has variables.
    def __make_input_space_1(self):
        # Make variable concept.
        make_sti_all(
            [self.paul, self.sally],
            sti_value_dict['IMPORTANT']
        )
        # Link with input space 1 type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.paul, self.sally],
            self.input_space_1,
            self.default_link_tv
        )

    # - Generic Space
    def __make_generic_space(self):
        # Link with human being type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [self.man, self.woman],
            self.human
        )

        """
        # Link with human being type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.grand_father, self.grand_mother,
                self.father, self.mother,
                self.son, self.daughter,
                self.paul, self.sally
            ],
            self.human
        )
        """

        # Link with man type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.grand_father, self.father,
                self.son,
                self.paul
            ],
            self.man
        )

        # Link with woman type.
        make_link_all(
            self.a,
            types.InheritanceLink,
            [
                self.grand_mother, self.mother,
                self.daughter,
                self.sally
            ],
            self.woman
        )

        # Link with generic space type.
        make_link_all(
            self.a,
            types.MemberLink,
            [self.man, self.woman, self.human],
            self.generic_space
        )

    # - Blended Space
    def __make_blend_space(self):
        pass

    def make(self):
        self.__make_atoms()
        self.__make_input_space_0()
        self.__make_input_space_1()
        self.__make_generic_space()
        self.__make_blend_space()

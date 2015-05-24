__author__ = 'DongMin Kim'

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import types
from opencog.type_constructors \
    import ConceptNode, VariableNode, \
    MemberLink, InheritanceLink

from util import blending_util
from util.blending_util import *


# Paul & Sally example in the book 'The Way We Think'
# It is a simplex network example.
class PaulSallyExample:
    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []
        self.atom_tv = TruthValue(0.9, 0.8)
        self.link_tv = TruthValue(0.7, 0.6)
        self.a_blend_target = \
            self.a.get_atoms_by_name(
                types.Atom,
                blending_util.BLEND_TARGET_NODE_NAME
            )[0]

    def _make_link_all(
            self,
            link_type,
            src_node_list,
            dst_node,
            tv
    ):
        for node in src_node_list:
            self.a.add_link(link_type, [node, dst_node], tv)

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

        # Link with blend target.
        self._make_link_all(
            types.MemberLink,
            [a_grand_father, a_grand_mother,
             a_father, a_mother, a_son, a_daughter],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with family frame type.
        self._make_link_all(
            types.MemberLink,
            [a_grand_father, a_grand_mother,
             a_father, a_mother, a_son, a_daughter],
            a_family,
            self.link_tv
        )

        # Link with input space 0 type.
        self._make_link_all(
            types.MemberLink,
            [a_father, a_daughter],
            a_input_space_0,
            self.link_tv
        )

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

    # - Input Space 1
    # Start to make 'actual human' network.
    # It not has frame. It has variables.
    def _make_input_space_1(self):
        # Make instance of space concept.
        a_input_space_1 = ConceptNode("InputSpace1", self.atom_tv)

        # Make variable concept.
        a_paul = VariableNode("Paul", TruthValue(0.9, 0.8))
        a_sally = VariableNode("Sally", TruthValue(0.9, 0.8))

        # Link with blend target.
        self._make_link_all(
            types.MemberLink,
            [a_paul, a_sally],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with input space 1 type.
        l_input_space_1_0 = MemberLink(a_paul, a_input_space_1, self.link_tv)
        l_input_space_1_1 = MemberLink(a_sally, a_input_space_1, self.link_tv)

        # Define new space by linking.
        l_space_1 = InheritanceLink(
            a_input_space_1,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.link_tv)

    # - Generic Space
    def _make_generic_space(self):
        # Make instance of space concept.
        a_generic_space = ConceptNode("GenericSpace", self.atom_tv)

        # Make role concept.
        a_human = ConceptNode("Human", self.atom_tv)
        a_man = ConceptNode("Man", self.atom_tv)
        a_woman = ConceptNode("Woman", self.atom_tv)

        # Link with blend target.
        self._make_link_all(
            types.MemberLink,
            [a_human, a_man, a_woman],
            self.a_blend_target,
            blend_target_link_tv
        )

        # Link with man type.
        self._make_link_all(
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
            a_man,
            self.link_tv
        )

        # Link with woman type.
        self._make_link_all(
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
            a_woman,
            self.link_tv
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
        l_human_0 = InheritanceLink(a_man, a_human, self.link_tv)
        l_human_1 = InheritanceLink(a_woman, a_human, self.link_tv)

        # Link with input space 0 type.
        self._make_link_all(
            types.MemberLink,
            [a_man, a_woman, a_human],
            a_generic_space,
            self.link_tv
        )

        # Define new space by linking.
        l_space_2 = InheritanceLink(
            a_generic_space,
            self.a.get_atoms_by_name(
                types.Atom, "Space")[0],
            self.link_tv)

    # - Blended Space
    def _make_blended_space(self):
        # Make instance of space concept.
        a_blended_space = ConceptNode("BlendedSpace", self.atom_tv)

    def make(self):
        self._make_input_space_0()
        self._make_input_space_1()
        self._make_generic_space()
        self._make_blended_space()

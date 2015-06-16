# coding=utf-8
from opencog.type_constructors import *
from opencog_b.python.conceptual_blending.blender.maker.base_maker import \
    BaseMaker
from opencog_b.python.conceptual_blending.util.blending_util import \
    get_weighted_tv, make_link_all

__author__ = 'DongMin Kim'


class MakeSimple(BaseMaker):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __make_atom_from_all(self, decided_atoms):
        """
        Choose all atoms.
        :param List decided_atoms: atoms list to make new atom.
        """
        # Make the new blend node.
        new_blend_atom_name = '('
        for atom in decided_atoms:
            new_blend_atom_name += str(atom.name)
            new_blend_atom_name += '-'

        new_blend_atom_name = new_blend_atom_name[0:-1]
        new_blend_atom_name += ')'

        self.ret = ConceptNode(
            new_blend_atom_name,
            get_weighted_tv(decided_atoms)
        )

        # Make the links between source nodes and newly blended node.
        # TODO: Give proper truth value, not random.
        # 랜덤 진릿값 말고 적당한 진릿값을 주어야 한다.
        make_link_all(
            self.a,
            types.AssociativeLink,
            decided_atoms,
            self.ret
        )

        # TODO: Give proper attention value.
        # new_blend_atom_name.av = {}

    def new_blend_make_impl(self, decided_atoms, config_base):
        self.__make_atom_from_all(decided_atoms)

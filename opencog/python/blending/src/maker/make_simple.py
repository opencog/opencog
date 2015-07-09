# coding=utf-8
from blending.util.blending_util import get_weighted_tv
from opencog.type_constructors import *

from blending.src.maker.base_maker import BaseMaker
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class MakeSimple(BaseMaker):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()

    def __make_atom_from_all(
            self, decided_atoms, atom_prefix, atom_separator, atom_postfix
    ):
        if len(decided_atoms) < 2:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        new_blend_atom_name = atom_prefix
        for atom in decided_atoms:
            new_blend_atom_name += str(atom.name)
            new_blend_atom_name += atom_separator

        new_blend_atom_name = new_blend_atom_name[0:-1]
        new_blend_atom_name += atom_postfix

        self.ret = self.a.add_node(
            types.ConceptNode,
            new_blend_atom_name,
            get_weighted_tv(decided_atoms)
        )

        # TODO: Give proper attention value.
        # new_blend_atom_name.av = {}

    def new_blend_make_impl(self, decided_atoms, config_base):
        # Make the new blend node.
        atom_prefix = BlendConfig().get_str(
            self.a, "make-atom-prefix", config_base
        )
        atom_separator = BlendConfig().get_str(
            self.a, "make-atom-separator", config_base
        )
        atom_postfix = BlendConfig().get_str(
            self.a, "make-atom-postfix", config_base
        )

        self.__make_atom_from_all(
            decided_atoms, atom_prefix, atom_separator, atom_postfix
        )

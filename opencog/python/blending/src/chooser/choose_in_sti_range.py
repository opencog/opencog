from opencog.atomspace import Handle
from opencog.type_constructors import types

from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    """Choose atoms within proper STI range.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "choose-sti-min", "1")
        BlendConfig().update(self.a, "choose-sti-max", "None")

    def __get_atoms_in_sti_range(
            self, focus_atoms, atom_type, least_count, sti_min, sti_max
    ):
        all_atoms = self.a.get_atoms_by_av(sti_min, sti_max)

        all_atoms_h_set = set(
            map(lambda atom: atom.handle_uuid(), all_atoms)
        )
        focus_atoms_h_set = set(
            map(lambda atom: atom.handle_uuid(), focus_atoms)
        )

        atoms_h_set = all_atoms_h_set & focus_atoms_h_set

        if len(atoms_h_set) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        self.ret = map(lambda atom_h: self.a[Handle(atom_h)], atoms_h_set)
        self.ret = filter(lambda atom: atom.is_a(atom_type), self.ret)
        if len(self.ret) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

    def atom_choose_impl(self, focus_atoms, config_base):
        atom_type = BlendConfig().get_str(
            self.a, "choose-atom-type", config_base
        )
        least_count = BlendConfig().get_int(
            self.a, "choose-least-count", config_base
        )
        sti_min = BlendConfig().get_str(self.a, "choose-sti-min", config_base)
        sti_max = BlendConfig().get_str(self.a, "choose-sti-max", config_base)

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node

        if least_count < 0:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        sti_min = int(sti_min) if sti_min.isdigit() else 1
        sti_max = int(sti_max) if sti_max.isdigit() else None

        self.__get_atoms_in_sti_range(
            focus_atoms, atom_type, least_count, sti_min, sti_max
        )

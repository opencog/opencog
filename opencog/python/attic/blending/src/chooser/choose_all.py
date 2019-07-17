from opencog.type_constructors import types

from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseAll(BaseChooser):
    """Atoms chooser that choosing all atoms.

    This chooser will choose every atom that has given type, and check the
    number of atoms.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()

    def __get_atoms_all(self, focus_atoms, atom_type, least_count):
        """Actual algorithm for choosing atoms.

        Args:
            focus_atoms: The atoms to blend.
            atom_type: The types to limit the result of chosen.
            least_count: Threshold value for minimum count of chosen atoms.
            :param focus_atoms: list[Atom]
            :param atom_type: int
            :param least_count: int
        """
        # Filter the atoms with specified type.
        self.ret = filter(
            lambda atom:
            atom.is_a(atom_type),
            focus_atoms
        )

        if len(self.ret) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

    def atom_choose_impl(self, focus_atoms, config_base):
        """Implemented factory method to choosing atoms.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        """
        # Choose all atoms in AtomSpace if focus_atoms was not given.
        focus_atoms = self.a.get_atoms_by_type(types.Atom) \
            if len(focus_atoms) is 0 \
            else focus_atoms

        atom_type = BlendConfig().get_str(
            self.a, "choose-atom-type", config_base
        )
        least_count = BlendConfig().get_int(
            self.a, "choose-least-count", config_base
        )

        # Check if given atom_type is valid or not.
        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node

        # Check if given least_count is valid or not.
        if least_count < 0:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Call the actual choosing algorithm method.
        self.__get_atoms_all(focus_atoms, atom_type, least_count)

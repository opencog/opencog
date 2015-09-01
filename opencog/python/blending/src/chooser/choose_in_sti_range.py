from opencog.atomspace import Handle
from opencog.type_constructors import types

from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    """Atoms chooser that choosing atoms within proper STI range.

    This chooser will choose every atom that has given type, within proper
    STI range, and check the number of atoms.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def make_default_config(self):
        """Initialize a default config for this class."""
        super(self.__class__, self).make_default_config()
        BlendConfig().update(self.a, "choose-sti-min", "1")
        BlendConfig().update(self.a, "choose-sti-max", "None")

    def __get_atoms_in_sti_range(
            self, focus_atoms, atom_type, least_count, sti_min, sti_max
    ):
        """Actual algorithm for choosing atoms.

        Args:
            focus_atoms: The atoms to blend.
            atom_type: The types to limit the result of chosen.
            least_count: Threshold value for minimum count of chosen atoms.
            sti_min: Threshold value for minimum value of STI.
            sti_max: Threshold value for maximum value of STI.
            :param focus_atoms: list[Atom]
            :param atom_type: int
            :param least_count: int
            :param sti_min: int
            :param sti_max: int
        """
        # Filter the atoms with specified type, and specified STI range.
        if sti_max is None:
            self.ret = filter(
                lambda atom:
                atom.is_a(atom_type) and
                sti_min <= atom.av["sti"],
                focus_atoms
            )
        else:
            self.ret = filter(
                lambda atom:
                atom.is_a(atom_type) and
                sti_min <= atom.av["sti"] < sti_max,
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
        atom_type = BlendConfig().get_str(
            self.a, "choose-atom-type", config_base
        )
        least_count = BlendConfig().get_int(
            self.a, "choose-least-count", config_base
        )
        sti_min = BlendConfig().get_str(self.a, "choose-sti-min", config_base)
        sti_max = BlendConfig().get_str(self.a, "choose-sti-max", config_base)

        # Check if given atom_type is valid or not.
        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node

        # Check if given least_count is valid or not.
        if least_count < 0:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Check if given range of STI value is valid or not.
        sti_min = int(sti_min) if sti_min.isdigit() else 1
        sti_max = int(sti_max) if sti_max.isdigit() else None

        # Call the actual choosing algorithm method.
        self.__get_atoms_in_sti_range(
            focus_atoms, atom_type, least_count, sti_min, sti_max
        )

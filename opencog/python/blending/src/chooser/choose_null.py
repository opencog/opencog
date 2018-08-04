from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseNull(BaseChooser):
    """Atoms chooser that does nothing.

    This chooser will returns given focus_atoms.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def atom_choose_impl(self, focus_atoms, config_base):
        """Implemented factory method to choosing atoms.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        """
        if focus_atoms is None or len(focus_atoms) < 2:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Just return same list.
        self.ret = focus_atoms

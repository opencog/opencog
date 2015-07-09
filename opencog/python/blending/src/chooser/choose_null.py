from blending.src.chooser.base_chooser import BaseChooser
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooseNull(BaseChooser):
    """Not choose, just return requested atoms.
    """

    def __init__(self, a):
        super(self.__class__, self).__init__(a)

    def __str__(self):
        return self.__class__.__name__

    def atom_choose_impl(self, focus_atoms, config_base):
        if focus_atoms is None or len(focus_atoms) < 2:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return

        # Just return same list.
        self.ret = focus_atoms

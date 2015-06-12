from blender_b.chooser.base_chooser import BaseChooser

__author__ = 'DongMin Kim'


class ChooseNull(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def atom_choose_impl(self, focus_atoms, config):
        if focus_atoms is None or len(focus_atoms) < 2:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        # Just return same list.
        self.ret = focus_atoms

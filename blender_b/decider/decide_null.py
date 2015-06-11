from blender_b.decider.base_decider import BaseDecider

__author__ = 'DongMin Kim'


class DecideNull(BaseDecider):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def __str__(self):
        return self.__class__.__name__

    def blending_decide_impl(self, a_chosen_atoms_list, config):
        if a_chosen_atoms_list is None or len(a_chosen_atoms_list) < 2:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        # Just return same list.
        self.ret = a_chosen_atoms_list

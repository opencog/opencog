from blending.util.blending_error import blending_status
from opencog.type_constructors import types
from blending.src.chooser.base_chooser import BaseChooser

__author__ = 'DongMin Kim'


class ChooseInSTIRange(BaseChooser):
    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)

    def make_default_config(self):
        self.default_config = {
            'choose-atom-type': 'Node',
            'choose-least-count': '2',
            'choose-sti-min': '1',
            'choose-sti-max': 'None'
        }

    def __get_atoms_in_sti_range(
            self, atom_type, least_count, sti_min, sti_max
    ):
        """
        Choose atoms within proper STI range.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        :param float sti_min: min value of sti to choose.
        :param float sti_max: max value of sti to choose.
        :return:
        """
        ret = []

        atoms = self.a.get_atoms_by_av(sti_min, sti_max)

        if len(atoms) < least_count:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            return ret

        found_atoms = filter(lambda atom: atom.is_a(atom_type), atoms)
        if len(found_atoms) < least_count:
            print('Size of requested list is too small.')
            return ret

        return found_atoms

    def __atom_choose_impl(self, config):
        atom_type = config.get('choose-atom-type')
        least_count = config.get('choose-least-count')
        sti_min = config.get('choose-sti-min')
        sti_max = config.get('choose-sti-max')

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node
        try:
            least_count = int(least_count)
            if least_count < 0:
                raise ValueError('Least count of atom list is too small.')
        except (TypeError, ValueError):
            least_count = 0
        try:
            sti_min = int(sti_min)
        except ValueError:
            sti_min = 1
        try:
            sti_max = int(sti_max)
        except ValueError:
            sti_max = None

        return self.__get_atoms_in_sti_range(
            atom_type, least_count, sti_min, sti_max
        )

    def atom_choose(self, config):
        if config is None:
            config = self.default_config

        return self.__atom_choose_impl(config)

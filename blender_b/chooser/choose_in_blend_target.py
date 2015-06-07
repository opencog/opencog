from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.blending_util import get_incoming_node_list, BlendTargetCtlForDebug
from util_b.general_util import BlConfig, BlLogger

__author__ = 'DongMin Kim'


class ChooseInBlendTarget(BaseChooser):
    # (BlendTarget is only useful in development phase)

    def __init__(self, atomspace):
        super(self.__class__, self).__init__(atomspace)
        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = [
            ['ATOM_TYPE', 'Atom'],
            ['LEAST_COUNT', '0'],
        ]
        BlConfig().make_default_config(str(self), default_config)

    def __get_atoms_in_blend_target(self, atom_type, least_count):
        """
        Choose atoms which are connected with 'BlendTarget' ConceptNode.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        :return:
        """
        ret = []

        a_atom_list = get_incoming_node_list(self.a, self.a_blend_target)

        if len(a_atom_list) < least_count:
            print('Size of atom list is too small')
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            return ret

        a_found_list = filter(lambda atom: atom.is_a(atom_type), a_atom_list)
        if len(a_found_list) < least_count:
            print('Size of requested list is too small.')
            return ret

        return a_found_list

    def __atom_choose_impl(self, config):
            atom_type = config.get('atom_type')
            least_count = config.get('least_count')

            if atom_type is None:
                atom_type = types.Atom
            if least_count is None:
                least_count = 0

            return self.__get_atoms_in_blend_target(atom_type, least_count)

    def atom_choose(self, config=None):
        if config is None:
            config = BlConfig().get_section(str(self))

        if BlConfig().is_use_blend_target:
            raise RuntimeError(
                "Can't use blend target! Please Check your config file."
            )

        return self.__atom_choose_impl(config)

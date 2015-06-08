from opencog.type_constructors import types
from blender_b.chooser.base_chooser import BaseChooser
from util_b.blending_util import get_incoming_node_list, BlendTargetCtlForDebug
from util_b.general_util import BlConfig

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
        default_config = {
            'ATOM_TYPE': 'Node',
            'LEAST_COUNT': '0',
        }
        BlConfig().make_default_config(str(self), default_config)

    def __get_atoms_in_blend_target(self, atom_type, least_count):
        """
        Choose atoms which are connected with 'BlendTarget' ConceptNode.
        :param Type atom_type: type of atoms to choose.
        :param int least_count: minimum number of atoms to choose.
        """
        a_atom_list = get_incoming_node_list(self.a, self.a_blend_target)

        if len(a_atom_list) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

        self.ret = filter(lambda atom: atom.is_a(atom_type), a_atom_list)
        if len(self.ret) < least_count:
            self.last_status = self.Status.NOT_ENOUGH_ATOMS
            raise UserWarning('Size of atom list is too small.')

    def atom_choose_impl(self, config):
        if config is None:
            config = BlConfig().get_section(str(self))

        if BlConfig().is_use_blend_target:
            self.last_status = self.Status.UNKNOWN_ERROR
            raise RuntimeError(
                "Can't use blend target! Please Check your config file."
            )

        atom_type = config.get('ATOM_TYPE')
        least_count = config.get('LEAST_COUNT')

        try:
            atom_type = types.__dict__[atom_type]
        except KeyError:
            atom_type = types.Node
        try:
            least_count = int(least_count)
        except (TypeError, ValueError):
            least_count = 0

        self.__get_atoms_in_blend_target(atom_type, least_count)

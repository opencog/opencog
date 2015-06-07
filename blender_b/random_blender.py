from base_blender import *
from blender_b.connector.connect_util import ConnectUtil
from util_b.blending_util import *
from blender_b.connector.connect_simple import *

__author__ = 'DongMin Kim'


class RandomBlender(BaseBlender):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

        self.config = None

        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None

        # TODO: change to works in several case, like 'link blending'.
        # Currently blender always makes new atom from 2 nodes.
        self.a_atom_0 = None
        self.a_atom_1 = None

        self.a_new_blended_atom = None

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = {
            'ATOMS_CHOOSER': 'ChooseInSTIRange',
            'LINK_CONNECTOR': 'ConnectSimple'
        }
        BlConfig().make_default_config(str(self), default_config)

    def prepare_hook(self, config):
        # BlLogger().log("Start RandomBlending")
        if config is None:
            config = BlConfig().get_section(str(self))
        self.config = config

        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None
        self.a_new_blended_atom = None

    def choose_atoms(self):
        self.chooser = self.chooser_finder.get_chooser(
            self.config.get('ATOMS_CHOOSER')
        )
        self.a_chosen_atoms_list = self.chooser.atom_choose()

    def decide_to_blend(self):
        # Currently, just check number of atoms.
        if self.a_chosen_atoms_list is None or \
           len(self.a_chosen_atoms_list) < 2:
            self.last_status = self.Status.NO_ATOMS_TO_BLEND
            raise UserWarning('No atoms to blend.')

        if len(self.a_chosen_atoms_list) < 2:
            self.last_status = self.Status.NO_ATOMS_TO_BLEND
            raise UserWarning('No atoms to blend.')

        a_index_list = random.sample(range(0, len(self.a_chosen_atoms_list)), 2)
        self.a_decided_atoms = [
            self.a_chosen_atoms_list[a_index_list[0]],
            self.a_chosen_atoms_list[a_index_list[1]]
        ]

    def init_new_blend_atom(self):
        # Make the blended node.
        self.a_atom_0 = self.a_decided_atoms[0]
        self.a_atom_1 = self.a_decided_atoms[1]
        self.a_new_blended_atom = ConceptNode(
            '(' +
            str(self.a_atom_0.name) + '_' + str(self.a_atom_1.name) +
            ')',
            rand_tv()
        )

    def connect_links(self):
        self.connector = self.connector_finder.get_connector(
            self.config.get('LINK_CONNECTOR')
        )

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

        # Make the links between exist nodes and newly blended node.
        # Adjust the attribute value of new links.

        # TODO: Optimize dst_info_container update period.
        # It should be move to out of src_node_list loop.
        for src_node in self.a_decided_atoms:
            src_info_cont = ConnectUtil().make_equal_link_containers(
                self.a, src_node
            )
            dst_info_cont = ConnectUtil().make_equal_link_containers(
                self.a, self.a_new_blended_atom
            )

            exclusive_link_set = src_info_cont.s - dst_info_cont.s
            non_exclusive_link_set = src_info_cont.s & dst_info_cont.s

            self.connector.add_new_links(
                src_info_cont, exclusive_link_set, self.a_new_blended_atom
            )
            self.connector.modify_exist_links(
                src_info_cont, dst_info_cont, non_exclusive_link_set
            )

        # Make the links between source nodes and newly blended node.
        # TODO: Change to make with proper reason, not make in every blending.
        make_link_all(
            self.a,
            types.AssociativeLink,
            self.a_decided_atoms,
            self.a_new_blended_atom
        )

    def finish_hook(self):
        # DEBUG: Link with blend target.
        if BlConfig().is_use_blend_target:
            self.a.add_link(
                types.MemberLink,
                [self.a_new_blended_atom, self.a_blend_target],
                blend_target_link_tv
            )

        # DEBUG: Link with Blended Space.
        a_blended_space = \
            self.a.get_atoms_by_name(
                types.Atom,
                "BlendedSpace"
            )[0]

        self.a.add_link(
            types.MemberLink,
            [self.a_new_blended_atom, a_blended_space],
            rand_tv()
        )

        BlLogger().log(
            str(self.a_new_blended_atom.h) +
            " " +
            str(self.a_new_blended_atom.name)
        )

        self.ret = self.a_new_blended_atom
        # BlLogger().log("Finish RandomBlending")

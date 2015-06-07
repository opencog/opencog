from blender_b.chooser.base_chooser import BaseChooser
from util_b.general_util import BlLogger

from base_blender import *
from util_b.blending_util import *
from util_b.link_copier import *

__author__ = 'DongMin Kim'


class RandomBlender(BaseBlender):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

        self.config = None
        self.link_copier_inst = LinkCopier(self.a)

        # TODO: change to works in several case, like 'link blending'.
        # Currently blender always makes new atom from 2 nodes.
        self.a_atom_0 = None
        self.a_atom_1 = None

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = [
            ['ATOMS_CHOOSER', 'ChooseInSTIRange']
        ]
        BlConfig().make_default_config(str(self), default_config)

    def prepare_hook(self, config):
        # BlLogger().log("Start RandomBlending")
        self.last_status = self.Status.IN_PROCESS
        if config is None:
            config = BlConfig().get_section(str(self))
        self.config = config

        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None
        self.a_new_blended_atom = None

        self.chooser = self.chooser_finder.get_chooser(
            self.config.get('ATOMS_CHOOSER')
        )

    def choose_atoms(self):
        self.a_chosen_atoms_list = self.chooser.atom_choose()

        if self.chooser.last_status != BaseChooser.Status.SUCCESS_CHOOSE:
            self.last_status = self.Status.ERROR_IN_CHOOSER
            return

    def decide_to_blend(self):
        # Currently, just check number of atoms.
        if len(self.a_chosen_atoms_list) < 2:
            BlLogger().log("No atoms to blend.")
            self.last_status = self.Status.NO_ATOMS_TO_BLEND
            return

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
        # Make the links between exist nodes and newly blended node.
        # Adjust the attribute value of new links.
        self.link_copier_inst.copy_all_link_to_new_node(
            [self.a_atom_0, self.a_atom_1],
            self.a_new_blended_atom
        )

        # Make the links between source nodes and newly blended node.
        # TODO: Change to make with proper reason, not make in every blending.
        self.a.add_link(
            types.AssociativeLink,
            [self.a_atom_0, self.a_new_blended_atom],
            rand_tv()
        )
        self.a.add_link(
            types.AssociativeLink,
            [self.a_atom_1, self.a_new_blended_atom],
            rand_tv()
        )

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

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

        self.last_status = self.Status.SUCCESS_BLEND
        # BlLogger().log("Finish RandomBlending")

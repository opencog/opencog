# coding=utf-8
from base_blender import *
from util_b.blending_util import *
from blender_b.connector.connect_simple import *

__author__ = 'DongMin Kim'


class NoRuleBlender(BaseBlender):
    def __init__(self, a):
        super(self.__class__, self).__init__(a)

        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

    def __str__(self):
        return self.__class__.__name__

    def make_default_config(self):
        default_config = {
            'ATOMS_CHOOSER': 'ChooseInSTIRange',
            'BLENDING_DECIDER': 'DecideBestSTI',
            'NEW_BLEND_ATOM_MAKER': 'MakeSimple',
            'LINK_CONNECTOR': 'ConnectSimple'
        }
        BlConfig().make_default_config(str(self), default_config)

    def prepare_hook(self, config):
        # BlLogger().log("Start RandomBlending")
        pass

    def create_chooser(self):
        self.chooser = self.chooser_finder.get_chooser(
            self.config.get('ATOMS_CHOOSER')
        )

    def create_decider(self):
        self.decider = self.decider_finder.get_decider(
            self.config.get('BLENDING_DECIDER')
        )

    def create_maker(self):
        self.maker = self.maker_finder.get_maker(
            self.config.get('NEW_BLEND_ATOM_MAKER')
        )

    def create_connector(self):
        self.connector = self.connector_finder.get_connector(
            self.config.get('LINK_CONNECTOR')
        )

    def finish_hook(self, a_new_blended_atom):
        # DEBUG: Link with blend target.
        if BlConfig().is_use_blend_target:
            self.a.add_link(
                types.MemberLink,
                [a_new_blended_atom, self.a_blend_target],
                blend_target_link_tv
            )

        # DEBUG: Link with Blended Space.
        a_blended_space = self.a.get_atoms_by_name(types.Atom, "BlendedSpace")[0]

        self.a.add_link(
            types.MemberLink,
            [a_new_blended_atom, a_blended_space],
            rand_tv()
        )

        BlLogger().log(
            str(a_new_blended_atom.h) +
            " " +
            str(a_new_blended_atom.name)
        )
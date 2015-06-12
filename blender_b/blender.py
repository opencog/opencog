from abc import ABCMeta, abstractmethod
from opencog.type_constructors import types
from blender_b.chooser.chooser_finder import ChooserFinder
from blender_b.connector.connector_finder import ConnectorFinder
from blender_b.decider.decider_finder import DeciderFinder
from blender_b.maker.maker_finder import MakerFinder
from util_b.blending_util import *
from util_b.general_util import enum_simulate, BlLogger, BlConfig

__author__ = 'DongMin Kim'


class Blender(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """
    __metaclass__ = ABCMeta

    Status = enum_simulate(
        'SUCCESS_BLEND',
        'IN_PROCESS',
        'UNKNOWN_ERROR',
        'ERROR_IN_PREPARE_HOOK'
        'ERROR_IN_CHOOSER',
        'ERROR_IN_DECIDER',
        'ERROR_IN_INIT_NEW_BLEND',
        'ERROR_IN_CONNECT_LINKS',
        'ERROR_IN_FINISH_HOOK',
        'NO_ATOMS_TO_BLEND'
    )

    def __init__(self, a):
        self.a = a
        self.last_status = self.Status.UNKNOWN_ERROR
        self.config = self.make_default_config()

        self.chooser_finder = ChooserFinder(self.a)
        self.connector_finder = ConnectorFinder(self.a)
        self.maker_finder = MakerFinder(self.a)
        self.decider_finder = DeciderFinder(self.a)

        self.chooser = None
        self.decider = None
        self.maker = None
        self.connector = None

        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None
        self.a_new_blended_atom = None
        self.ret = None

    def __str__(self):
        return self.__class__.__name__

    def is_succeeded(self):
        return (lambda x: True
                if x == Blender.Status.SUCCESS_BLEND
                else False
                )(self.last_status)

    def make_default_config(self):
        default_config = {
            'ATOMS_CHOOSER': 'ChooseInSTIRange',
            'BLENDING_DECIDER': 'DecideBestSTI',
            'NEW_BLEND_ATOM_MAKER': 'MakeSimple',
            'LINK_CONNECTOR': 'ConnectSimple'
        }
        BlConfig().make_default_config(str(self), default_config)
        return default_config

    """
    Define template blending method.
    (Option) Prepare for blend before start blending.
    1. Choose atoms
    2. Decide to blend
    3. Initialize new blend atom
    4. Connect links to new blend atom from exist atom
    (Option) Finish rest of blending.
    """
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
    """
    End of define.
    """

    def blend(self, config=None):
        self.last_status = self.Status.IN_PROCESS
        self.ret = None

        self.config = config
        if self.config is None:
            self.config = BlConfig().get_section(str(self))

        try:
            self.create_chooser()
            self.create_decider()
            self.create_maker()
            self.create_connector()

            # Give interface to each blenders to suitable prepare works.
            # eg. caching, ...
            self.prepare_hook(config)

            # Choose nodes to blending.
            self.a_chosen_atoms_list = self.chooser.atom_choose()

            # Decide whether or not to execute blending and prepare.
            self.a_decided_atoms = \
                self.decider.blending_decide(self.a_chosen_atoms_list)

            # Initialize the new blend node.
            self.a_new_blended_atom = \
                self.maker.new_blend_make(self.a_decided_atoms)

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.connector.link_connect(
                self.a_decided_atoms, self.a_new_blended_atom
            )

            # Give interface to each blenders to finish works.
            self.finish_hook(self.a_new_blended_atom)

            # If all task finished successfully, save new atom to return.
            self.ret = self.a_new_blended_atom

        except UserWarning as e:
            BlLogger().log("Skipping blend, caused by '" + str(e) + "'")
            BlLogger().log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_BLEND

        return self.ret

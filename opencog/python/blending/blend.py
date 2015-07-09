import random

from opencog.atomspace import TruthValue
from opencog.type_constructors import types, ConceptNode
from opencog.logger import log

from blending.src.chooser.chooser_finder import ChooserFinder
from blending.src.decider.decider_finder import DeciderFinder
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status, get_status_str
from blending.util.link_copier import LinkCopier


__author__ = 'DongMin Kim'

class ConceptualBlending:

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()

        self.config_base = None
        self.focus_atoms = []
        self.link_copier_inst = LinkCopier(self.a)

        self.chooser = None
        self.decider = None

        self.chosen_atoms = []
        self.decided_atoms = []
        self.blended_atom = None
        self.merged_atom = None
        self.blended_atoms = []

    def make_default_config(self):
        BlendConfig().update(self.a, "atoms-chooser", "ChooseInSTIRange")
        BlendConfig().update(self.a, "blending-decider", "DecideBestSTI")

    def __prepare(self, focus_atoms, config_base):
        # Prepare blending.
        self.last_status = blending_status.IN_PROCESS

        if type(config_base) is list:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning("Config can't be list type.")

        default_config_base = self.a.add_node(
            types.ConceptNode,
            BlendConfig.config_prefix
        )

        self.config_base = default_config_base \
            if config_base is None \
            else config_base
        self.focus_atoms = self.a.get_atoms_by_type(types.Node) \
            if focus_atoms is None \
            else focus_atoms

        self.__make_workers()

    def __make_workers(self):
        self.chooser = ChooserFinder(self.a).get_chooser(self.config_base)
        self.decider = DeciderFinder(self.a).get_decider(self.config_base)

    def new_blend_make(self):
        # Make a blended node.
        self.merged_atom = ConceptNode(
            str(self.decided_atoms[0].name) + '-' +
            str(self.decided_atoms[1].name),
            TruthValue(
                random.uniform(0.5, 0.9),
                random.uniform(0.5, 0.9)
            )
        )

    def link_connect(self):
        # Make the links between exist nodes and newly blended node.
        # Adjust the attribute value of new links.
        self.link_copier_inst.copy_all_link_to_new_node(
            [self.decided_atoms[0], self.decided_atoms[1]],
            self.merged_atom
        )

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

    def __clean_up(self):
        self.last_status = blending_status.SUCCESS

    def run(self, focus_atoms=None, config_base=None):
        self.__prepare(focus_atoms, config_base)

        try:
            # Select nodes to blending.
            self.chosen_atoms = \
                self.chooser.atom_choose(self.focus_atoms, self.config_base)

            # Decide whether or not to execute blending and prepare.
            self.decided_atoms = \
                self.decider.blending_decide(self.focus_atoms, self.config_base)

            # Initialize the new blend node.
            self.new_blend_make()

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.link_connect()

        except UserWarning as e:
            log.info("Skipping blend, caused by '" + str(e) + "'")
            log.info(
                "Last status is '" +
                get_status_str(self.last_status) +
                "'"
            )

        # Sum up blending.
        self.__clean_up()

        return self.merged_atom

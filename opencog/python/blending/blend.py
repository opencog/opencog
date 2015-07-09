from opencog.type_constructors import types, ConceptNode
from opencog.logger import log

from blending.src.chooser.chooser_finder import ChooserFinder
from blending.src.decider.decider_finder import DeciderFinder
from blending.src.maker.maker_finder import MakerFinder
from blending.src.connector.connector_finder import ConnectorFinder

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status, get_status_str

__author__ = 'DongMin Kim'


class ConceptualBlending:

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()

        self.config_base = None
        self.focus_atoms = None

        self.chosen_atoms = []
        self.decided_atoms = []
        self.blended_atom = None
        self.merged_atom = None
        self.blended_atoms = []

    def make_default_config(self):
        BlendConfig().update(self.a, "atoms-chooser", "ChooseAll")
        BlendConfig().update(self.a, "blending-decider", "DecideBestSTI")
        BlendConfig().update(self.a, "new-blend-atom-maker", "MakeSimple")
        BlendConfig().update(self.a, "link-connector", "ConnectSimple")

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

    def __clean_up(self):
        self.last_status = blending_status.SUCCESS

    def run(self, focus_atoms=None, config_base=None):
        self.__prepare(focus_atoms, config_base)

        try:
            # Select nodes to blending.
            self.chosen_atoms = \
                ChooserFinder(self.a).\
                get_chooser(self.config_base).\
                atom_choose(self.focus_atoms, self.config_base)

            # Decide whether or not to execute blending and prepare.
            self.decided_atoms = \
                DeciderFinder(self.a).\
                get_decider(self.config_base).\
                blending_decide(self.chosen_atoms, self.config_base)

            # Initialize the new blend node.
            self.merged_atom = \
                MakerFinder(self.a).\
                get_maker(self.config_base).\
                new_blend_make(self.decided_atoms, self.config_base)

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.blended_atoms = \
                ConnectorFinder(self.a).\
                get_connector(self.config_base).\
                link_connect(self.decided_atoms, self.merged_atom, config_base)

        except UserWarning as e:
            log.info("Skipping blend, caused by '" + str(e) + "'")
            log.info(
                "Last status is '" +
                get_status_str(self.last_status) +
                "'"
            )

        # Sum up blending.
        self.__clean_up()

        return self.blended_atoms

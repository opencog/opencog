from opencog.type_constructors import types
from opencog.logger import log

from blending.src.chooser.chooser_finder import ChooserFinder
from blending.src.decider.decider_finder import DeciderFinder
from blending.src.maker.maker_finder import MakerFinder
from blending.src.connector.connector_finder import ConnectorFinder

from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ConceptualBlending:
    """Conceptual Blending class.

    Conceptual Blending is a process that creates new concepts via judiciously
    combining pieces of old concepts. It makes new concept by blends two or
    more ConceptNodes into a new one.

    Algorithm procedure:
    *. Prepare for blend before start blending.
    1. Choose atoms to blend(can skip if focus atom was given)
    2. Decide to blend(can skip if focus atom was given)
    3. Initialize new blend atom
    4. Connect links to new blend atom from exist atoms
    *. Clean up the blending.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        focus_atoms: The atoms to blend.
        config_base: A Node to save custom config.
        chosen_atoms: The chosen atoms made by atoms chooser.
        decided_atoms: The decided atoms made by blending decider.
        merged_atom: A merged atom made by new atom maker.
        blended_atoms: The blended atoms made by link connector.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type focus_atoms: list[Atom]
        :type config_base: Node
        :type chosen_atoms: list[Atom]
        :type decided_atoms: list[Atom]
        :type merged_atom: Atom
        :type blended_atoms: list[Atom]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()

        self.focus_atoms = None
        self.config_base = None

        self.chosen_atoms = []
        self.decided_atoms = []
        self.merged_atom = None
        self.blended_atoms = []

    def make_default_config(self):
        """Initialize a default config for this class."""
        BlendConfig().update(self.a, "atoms-chooser", "ChooseAll")
        BlendConfig().update(self.a, "blending-decider", "DecideBestSTI")
        BlendConfig().update(self.a, "new-blend-atom-maker", "MakeSimple")
        BlendConfig().update(self.a, "link-connector", "ConnectSimple")

    def __prepare(self, focus_atoms, config_base):
        """Prepare a blending.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        """
        self.last_status = blending_status.IN_PROCESS

        if type(config_base) is list:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning("Config can't be list type.")

        default_config_base = self.a.add_node(
            types.ConceptNode,
            BlendConfig().config_prefix_name
        )

        self.config_base = default_config_base \
            if config_base is None \
            else config_base
        self.focus_atoms = [] \
            if focus_atoms is None \
            else focus_atoms

    def __clean_up(self):
        """Clean up blending."""
        self.last_status = blending_status.SUCCESS

    def run(self, focus_atoms=None, config_base=None):
        """Execute a conceptual blending algorithm.

        Args:
            focus_atoms: The atoms to blend.
            config_base: A Node to save custom config.
            :param focus_atoms: list[Atom]
            :param config_base: Atom
        Returns:
            The blended atom(s).
            Example:
            [(ConceptNode "car-man"),
             (ConceptNode "man-car"),
             ...]
            If a list is empty, then means blender couldn't make a proper
            blend atom(s) with given atoms.
            :rtype : list[Atom]
        """

        try:
            self.__prepare(focus_atoms, config_base)

            # Choose nodes to blending.
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

            # Sum up blending.
            self.__clean_up()
        except UserWarning as e:
            log.info('Skip blending due to: ' + str(e))
            self.blended_atoms = []

        # Returns the blended atom(s).
        return self.blended_atoms

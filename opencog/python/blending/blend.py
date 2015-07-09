import random
from blending.src.chooser.chooser_finder import ChooserFinder
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status, get_status_str
from opencog.atomspace import TruthValue
from opencog.type_constructors import types, ConceptNode
from blending.util.link_copier import LinkCopier

__author__ = 'DongMin Kim'

class ConceptualBlending:

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.make_default_config()

        self.config_base = None
        self.link_copier_inst = LinkCopier(self.a)

        self.chooser_finder = ChooserFinder(self.a)
        self.chooser = None

        self.chosen_atoms = []
        self.decided_atoms = []
        self.blended_atom = None
        self.merged_atom = None
        self.blended_atoms = []

    def make_default_config(self):
        BlendConfig().update(self.a, "atoms-chooser", "ChooseAll")

    def __prepare(self, config):
        # Prepare blending.
        self.last_status = blending_status.IN_PROCESS
        self.config_base = self.decided_atoms if config is None else config
        self.__make_workers()

    def __make_workers(self):
        self.chooser = self.chooser_finder.get_chooser(
            BlendConfig().get_str(self.a, "atoms-chooser", self.config_base)
        )

    def blending_decide(self):
        # Currently, just check number of atoms.
        if len(self.chosen_atoms) < 2:
            self.last_status = blending_status.NOT_ENOUGH_ATOMS
            raise UserWarning(get_status_str(self.last_status))

        a_index_list = random.sample(range(0, len(self.chosen_atoms)), 2)
        self.decided_atoms = [
            self.chosen_atoms[a_index_list[0]],
            self.chosen_atoms[a_index_list[1]]
        ]

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

    def run(self, config=None):
        self.__prepare(config)

        # Select nodes to blending.
        self.chosen_atoms = self.chooser.atom_choose(self.config_base)

        # Decide whether or not to execute blending and prepare.
        self.blending_decide()

        # Initialize the new blend node.
        self.new_blend_make()

        # Make the links between exist nodes and newly blended node.
        # Check the severe conflict links in each node and remove.
        # Detect and improve conflict links in newly blended node.
        self.link_connect()

        # Sum up blending.
        self.__clean_up()

        return self.merged_atom

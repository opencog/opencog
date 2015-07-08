import random
from blending.src.chooser.chooser_finder import ChooserFinder
from blending.util.blending_error import blending_status, get_status_str
from opencog.atomspace import TruthValue
from opencog.type_constructors import types, ConceptNode
from opencog.logger import log
from blending.util.link_copier import LinkCopier

__author__ = 'DongMin Kim'

class ConceptualBlending:

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR
        self.default_config = None
        self.make_default_config()

        self.config = None
        self.link_copier_inst = LinkCopier(self.a)

        self.chooser_finder = ChooserFinder(self.a)
        self.chooser = None

        self.chosen_atoms = None
        self.decided_atoms = None
        self.new_blended_atom = None

    def make_default_config(self):
        self.default_config = {
            'atoms-chooser': 'ChooseInSTIRange'
        }

    def __prepare(self, config):
        # Prepare blending.
        self.last_status = blending_status.IN_PROCESS
        self.config = self.decided_atoms if config is None else config
        self.__make_workers()

    def __make_workers(self):
        chooser_name = self.config['atoms-chooser']
        self.chooser = self.chooser_finder.get_chooser(chooser_name)
        if self.chooser is None:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning(
                "Can't find " + chooser_name + "\n" +
                "Available choosers: " + "\n" +
                str(self.chooser_finder.chooser_list)
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
        self.new_blended_atom = ConceptNode(
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
            self.new_blended_atom
        )

        # Detect and improve conflict links in newly blended node.
        # - Do nothing.

    def __clean_up(self):
        self.last_status = blending_status.SUCCESS

    def run(self, config):
        self.__prepare(config)

        # Select nodes to blending.
        self.chosen_atoms = self.chooser.atom_choose({
            'choose-atom-type': 'Node',
            'choose-least-count': '2',
            'choose-sti-min': '1',
            'choose-sti-max': 'None'
        })

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

        return self.new_blended_atom

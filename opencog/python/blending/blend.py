from opencog.type_constructors import *

from blending.src.chooser.chooser_finder import ChooserFinder
from blending.src.connector.connector_finder import ConnectorFinder
from blending.src.decider.decider_finder import DeciderFinder
from blending.src.maker.maker_finder import MakerFinder
from blending.util.blend_logger import blend_log
from blending.util.blend_config import BlendConfig
from blending.util.general_util import enum_simulate

__author__ = 'DongMin Kim'


class ConceptualBlending:
    """Conceptual Blending Class.

    TODO: Write detailed description.

    Attributes:
        a: An instance of atomspace.
        blender_finder:
        blender:
        :type a: opencog.atomspace_details.AtomSpace
    """

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
        """
        Args:
            a: An instance of atomspace.
            :param a: opencog.atomspace.AtomSpace
        """

        self.a = a
        self.last_status = self.Status.UNKNOWN_ERROR
        self.make_default_config()

        self.chooser_finder = ChooserFinder(self.a)
        self.connector_finder = ConnectorFinder(self.a)
        self.maker_finder = MakerFinder(self.a)
        self.decider_finder = DeciderFinder(self.a)

        self.chooser = None
        self.decider = None
        self.maker = None
        self.connector = None

        self.chosen_atoms = None
        self.decided_atoms = None
        self.new_blended_atom = None
        self.new_blended_atoms = None
        self.ret = None

    def __str__(self):
        return self.__class__.__name__

    def is_succeeded(self):
        return (lambda x: True
                if x == ConceptualBlending.Status.SUCCESS_BLEND
                else False
                )(self.last_status)

    def make_default_config(self):
        BlendConfig().update(self.a, "atoms-chooser", "ChooseAll")
        BlendConfig().update(self.a, "blending-decider", "DecideBestSTI")
        BlendConfig().update(self.a, "new-blend-atom-maker", "MakeSimple")
        BlendConfig().update(self.a, "link-connector", "ConnectSimple")

    def prepare_hook(self, config_base):
        pass

    def make_workers(self, config_base):
        self.chooser = self.chooser_finder.get_chooser(
            BlendConfig().get_str(self.a, "atoms-chooser", config_base)
        )
        self.decider = self.decider_finder.get_decider(
            BlendConfig().get_str(self.a, "blending-decider", config_base)
        )
        self.maker = self.maker_finder.get_maker(
            BlendConfig().get_str(self.a, "new-blend-atom-maker", config_base)
        )
        self.connector = self.connector_finder.get_connector(
            BlendConfig().get_str(self.a, "link-connector", config_base)
        )

    def finish_hook(self, new_blended_atom, config_base):
        pass

    """
    Define template blending method.
    (Option) Prepare for blend before start blending.
    1. Choose atoms
    2. Decide to blend
    3. Initialize new blend atom
    4. Connect links to new blend atom from exist atom
    (Option) Finish rest of blending.
    """
    def run(self, focus_atoms=None, config_base=None):
        """Execute conceptual blending.new_blended_atoms

        TODO: Write detailed description.

        Args:
            focus_atoms:
            config_base:
            :param focus_atoms: list[Atom]
            :param config_base: Atom

        Returns:
            The blended atom(s). For example:

            [(ConceptNode "car-man"),
             (ConceptNode "man-car"),
             ...]

            If a list is empty, then blender can't find proper blend atoms.
            :rtype : list[Atom]
        """
        self.last_status = self.Status.IN_PROCESS
        self.ret = None

        if focus_atoms is None:
            focus_atoms = []

        if config_base is None:
            config_base = self.a.add_node(types.ConceptNode, BlendConfig().name)

        try:
            self.make_workers(config_base)

            # Give interface to each blenders to suitable prepare works.
            # eg. caching, ...
            self.prepare_hook(config_base)

            # Choose nodes to blending.
            self.chosen_atoms = \
                self.chooser.atom_choose(focus_atoms, config_base)

            # Decide whether or not to execute blending and prepare.
            self.decided_atoms = \
                self.decider.blending_decide(self.chosen_atoms, config_base)

            # Initialize the new blend node.
            self.new_blended_atom = \
                self.maker.new_blend_make(self.decided_atoms, config_base)

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.new_blended_atoms = self.connector.link_connect(
                self.decided_atoms, self.new_blended_atom, config_base
            )

            # Give interface to each blenders to finish works.
            self.finish_hook(self.new_blended_atoms, config_base)

            # If all task finished successfully, save new atoms to return.
            self.ret = self.new_blended_atoms

        except UserWarning as e:
            blend_log("Skipping blend, caused by '" + str(e) + "'")
            blend_log(
                "Last status is '" +
                self.Status.reverse_mapping[self.last_status] +
                "'"
            )

        if self.last_status == self.Status.IN_PROCESS:
            self.last_status = self.Status.SUCCESS_BLEND

        if self.ret is None:
            self.ret = []

        return self.ret

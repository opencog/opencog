from blender_b.chooser.chooser_finder import ChooserFinder
from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig, enum_simulate

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod


class BaseBlender(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """

    Status = enum_simulate(
        'SUCCESS_BLEND',
        'IN_PROCESS',
        'UNKNOWN_ERROR',
        'ERROR_IN_CHOOSER',
        'ERROR_IN_DECIDER',
        'NO_ATOMS_TO_BLEND'
    )

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        self.last_status = self.Status.UNKNOWN_ERROR
        self.make_default_config()

        self.chooser_finder = ChooserFinder(self.a)

        self.chooser = None

        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None
        self.a_new_blended_atom = None

    def __str__(self):
        return self.__class__.__name__

    def get_last_status(self):
        return self.last_status

    def make_default_config(self):
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
    def prepare_hook(self, config):
        self.last_status = self.Status.IN_PROCESS

    @abstractmethod
    def choose_atoms(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def decide_to_blend(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def init_new_blend_atom(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def connect_links(self):
        raise NotImplementedError("Please implement this method.")

    def finish_hook(self):
        self.last_status = self.Status.SUCCESS_BLEND
    """
    End of define.
    """

    def blend(self, config=None):
        # Give interface to each blenders to suitable prepare works.
        # eg. caching, ...
        self.prepare_hook(config)

        # Choose nodes to blending.
        self.choose_atoms()

        # Decide whether or not to execute blending and prepare.
        self.decide_to_blend()
        if self.last_status == self.Status.NO_ATOMS_TO_BLEND:
            return

        # Initialize the new blend node.
        self.init_new_blend_atom()

        # Make the links between exist nodes and newly blended node.
        # Check the severe conflict links in each node and remove.
        # Detect and improve conflict links in newly blended node.
        self.connect_links()

        # Give interface to each blenders to finish works.
        self.finish_hook()


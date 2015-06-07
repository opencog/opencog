from blender_b.chooser.chooser_factory import ChooserFactory
from util_b.blending_util import BlendTargetCtlForDebug
from util_b.general_util import BlConfig, BlLogger

__author__ = 'DongMin Kim'

from abc import ABCMeta, abstractmethod


class BaseBlender(object):
    """
    :type a: opencog.atomspace_details.AtomSpace
    """

    SUCCESS_BLEND = 0
    IN_PROCESS = 1

    UNKNOWN_ERROR = 10
    ERROR_IN_CHOOSER = 11
    ERROR_IN_DECIDER = 12

    NO_ATOMS_TO_BLEND = 20

    __metaclass__ = ABCMeta

    def __init__(self, a):
        self.a = a
        if BlConfig().is_use_blend_target:
            self.a_blend_target = BlendTargetCtlForDebug().get_blend_target()

        self.chooser_factory = ChooserFactory(self.a)

        self.last_status = self.UNKNOWN_ERROR
        self.a_chosen_atoms_list = None
        self.a_decided_atoms = None
        self.a_new_blended_atom = None

    def __str__(self):
        return 'BaseBlender'

    def get_last_status(self):
        return self.last_status

    def prepare_hook(self):
        self.last_status = self.IN_PROCESS

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
        self.last_status = self.SUCCESS_BLEND

    def blend(self):
        # Give interface to each blenders to suitable prepare works.
        # eg. caching, ...
        self.prepare_hook()

        # Select nodes to blending.
        self.choose_atoms()

        # Decide whether or not to execute blending and prepare.
        self.decide_to_blend()
        if self.last_status == self.NO_ATOMS_TO_BLEND:
            return

        # Initialize the new blend node.
        self.init_new_blend_atom()

        # Make the links between exist nodes and newly blended node.
        # Check the severe conflict links in each node and remove.
        # Detect and improve conflict links in newly blended node.
        self.connect_links()

        # Give interface to each blenders to finish works.
        self.finish_hook()


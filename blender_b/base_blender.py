from abc import ABCMeta, abstractmethod
from blender_b.chooser.chooser_finder import ChooserFinder
from blender_b.connector.connector_finder import ConnectorFinder
from blender_b.decider.decider_finder import DeciderFinder
from util_b.general_util import enum_simulate, BlLogger

__author__ = 'DongMin Kim'


class BaseBlender(object):
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
        self.make_default_config()

        self.chooser_finder = ChooserFinder(self.a)
        self.connector_finder = ConnectorFinder(self.a)
        self.decider_finder = DeciderFinder(self.a)

        self.chooser = None
        self.connector = None
        self.decider = None

        self.ret = None
        self.make_default_config()

    def __str__(self):
        return self.__class__.__name__

    def is_succeeded(self):
        return (lambda x: True
                if x == BaseBlender.Status.SUCCESS_BLEND
                else False
                )(self.last_status)

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
        pass

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
        pass
    """
    End of define.
    """

    def blend(self, config=None):
        self.last_status = self.Status.IN_PROCESS
        self.ret = None

        try:
            # Give interface to each blenders to suitable prepare works.
            # eg. caching, ...
            self.prepare_hook(config)

            # Choose nodes to blending.
            self.choose_atoms()

            # Decide whether or not to execute blending and prepare.
            self.decide_to_blend()

            # Initialize the new blend node.
            self.init_new_blend_atom()

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.connect_links()

            # Give interface to each blenders to finish works.
            self.finish_hook()
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

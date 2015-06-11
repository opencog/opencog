from abc import ABCMeta, abstractmethod
from blender_b.chooser.chooser_finder import ChooserFinder
from blender_b.connector.connector_finder import ConnectorFinder
from blender_b.decider.decider_finder import DeciderFinder
from blender_b.maker.maker_finder import MakerFinder
from util_b.general_util import enum_simulate, BlLogger, BlConfig

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
        self.config = self.make_default_config()

        self.chooser_finder = ChooserFinder(self.a)
        self.connector_finder = ConnectorFinder(self.a)
        self.maker_finder = MakerFinder(self.a)
        self.decider_finder = DeciderFinder(self.a)

        self.chooser = None
        self.decider = None
        self.maker = None
        self.connector = None

        self.ret = None

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
    @abstractmethod
    def prepare_hook(self, config):
        pass

    @abstractmethod
    def create_chooser(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def create_decider(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def create_maker(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def create_connector(self):
        raise NotImplementedError("Please implement this method.")

    @abstractmethod
    def finish_hook(self, a_new_blended_atom):
        pass
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
            a_chosen_atoms_list = self.chooser.atom_choose()

            # Decide whether or not to execute blending and prepare.
            a_decided_atoms = self.decider.blending_decide(a_chosen_atoms_list)

            # Initialize the new blend node.
            a_new_blended_atom = self.maker.new_blend_make(a_decided_atoms)

            # Make the links between exist nodes and newly blended node.
            # Check the severe conflict links in each node and remove.
            # Detect and improve conflict links in newly blended node.
            self.connector.link_connect(a_decided_atoms, a_new_blended_atom)

            # Give interface to each blenders to finish works.
            self.finish_hook(a_new_blended_atom)

            # If all task finished successfully, save new atom to return.
            self.ret = a_new_blended_atom

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

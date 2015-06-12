import opencog.cogserver
from opencog.utilities import *
from blender_b.blender import Blender
from util_b.experiment_codes import ExperimentCodes
from util_b.general_util import BlLogger

__author__ = 'DongMin Kim'


class ConceptualBlending:
    """Conceptual Blending Class.

    TODO: Write detailed description.

    Attributes:
        a: An instance of atomspace.
        blender_finder:
        blender:
        :type a: AtomSpace
        :type blender: Blender
    """

    def __init__(self, a):
        """
        Args:
            a: An instance of atomspace.
            :param a: AtomSpace
        """

        # TODO: Remove atomspace instance in Conceptual Blending tool.
        # Blending do not always need atomspace. If anywhere exists that
        # needs atomspace, there should prepare check functions by themselves.
        # If blending function was not called in python environment, how to
        # check if python's atomspace was initialized?
        self.a = a
        self.blender = None

    def run(self, target_atoms=None, config=None):
        """Execute conceptual blending.

        TODO: Write detailed description.

        Args:
            target_atoms:
            config:
            :param target_atoms: list[Atom]
            :param config: Atom

        Returns:
            The blended atom(s). For example:

            [(ConceptNode "car-man"),
             (ConceptNode "man-car"),
             ...]

            If a list is empty, then blender can't find proper blend atoms.
            :rtype : list[Atom]
        """
        self.blender = Blender(self.a)
        self.blender.blend()

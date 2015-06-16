from opencog.type_constructors import *
from blender.blender import Blender
from opencog_b.python.conceptual_blending.util.general_util import BlLogger
from util.general_util import BlAtomConfig

__author__ = 'DongMin Kim'


class ConceptualBlending:
    """Conceptual Blending Class.

    TODO: Write detailed description.

    Attributes:
        a: An instance of atomspace.
        blender_finder:
        blender:
        :type a: opencog.atomspace.AtomSpace
        :type blender: Blender
    """

    def __init__(self, a):
        """
        Args:
            a: An instance of atomspace.
            :param a: opencog.atomspace.AtomSpace
        """

        # TODO: Remove atomspace instance in Conceptual Blending tool.
        # Blending do not always need atomspace. If anywhere exists that
        # needs atomspace, there should prepare check functions by themselves.
        # If blending function was not called in python environment, how to
        # check if python's atomspace was initialized?
        self.a = a
        self.blender = None

    def run(self, focus_atoms=None, config_base=None):
        """Execute conceptual blending.

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
        
        if config_base is None:
            config_base = self.a.add_node(types.ConceptNode, BlAtomConfig().name)
        else:
            # TODO: If config_base exists, enroll custom config_base.
            # BlAtomConfig().add(self.a, "blender", "RuleBlender")
            pass

        BlLogger().change_config(self.a)
        self.blender = Blender(self.a)
        return self.blender.blend(focus_atoms, config_base)

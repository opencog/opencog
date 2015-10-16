from blending.src.chooser.choose_null import ChooseNull
from blending.src.chooser.choose_all import ChooseAll
from blending.src.chooser.choose_in_sti_range import ChooseInSTIRange
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    """Provider class to make atoms chooser instance.

    This provider will made the instance of atoms chooser, and returns them to
    the blender.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        choosers: An available atoms chooser list.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type choosers: dict[BaseChooser]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.choosers = {
            ChooseNull.__name__: ChooseNull,
            ChooseAll.__name__: ChooseAll,
            ChooseInSTIRange.__name__: ChooseInSTIRange
        }

    def get_chooser(self, config_base):
        """Provider method for atoms chooser.

        Args:
            config_base: A Node to save custom config.
            :param config_base: Atom
        Returns:
            The instance of atoms chooser.
            :rtype : BaseChooser
        Raises:
            UserWarning: Can't find the atoms chooser with given name.
        """
        self.last_status = blending_status.IN_PROCESS

        chooser = self.choosers.get(
            BlendConfig().get_str(self.a, "atoms-chooser", config_base)
        )
        if chooser is not None:
            self.last_status = blending_status.SUCCESS
            return chooser(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning

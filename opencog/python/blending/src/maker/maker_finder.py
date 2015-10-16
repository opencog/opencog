from blending.src.maker.make_simple import MakeSimple
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class MakerFinder(object):
    """Provider class to make atom maker instance.

    This provider will made the instance of atom maker, and returns them to the
    blender.

    Attributes:
        a: An instance of AtomSpace.
        last_status: A last status of class.
        makers: An available new blend atom makers list.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type makers: dict[BaseMaker]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.makers = {
            MakeSimple.__name__: MakeSimple
        }

    def get_maker(self, config_base):
        """Provider method for atom maker.

        Args:
            config_base: A Node to save custom config.
            :param config_base: Atom
        Returns:
            The instance of atom maker.
            :rtype : BaseMaker
        Raises:
            UserWarning: Can't find the atom maker with given name.
        """
        self.last_status = blending_status.IN_PROCESS

        maker = self.makers.get(
            BlendConfig().get_str(self.a, "new-blend-atom-maker", config_base)
        )
        if maker is not None:
            self.last_status = blending_status.SUCCESS
            return maker(self.a)
        else:
            self.last_status = blending_status.PARAMETER_ERROR
            raise UserWarning

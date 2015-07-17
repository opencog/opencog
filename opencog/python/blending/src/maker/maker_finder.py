from blending.src.maker.make_simple import MakeSimple
from blending.util.blending_config import BlendConfig
from blending.util.blending_error import blending_status

__author__ = 'DongMin Kim'


class MakerFinder(object):
    """Provide maker instance for user.

    Attributes:
        a: An instance of atomspace.
        last_status: A last status of class.
        makers: An available new blend atom makers list.
        :type a: opencog.atomspace.AtomSpace
        :type last_status: int
        :type makers: dict[abc.ABCMeta]
    """

    def __init__(self, a):
        self.a = a
        self.last_status = blending_status.UNKNOWN_ERROR

        self.makers = {
            MakeSimple.__name__: MakeSimple
        }

    def get_maker(self, config_base):
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

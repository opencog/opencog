from blender_b.maker.make_simple import MakeSimple
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class MakerFinder(object):
    def __init__(self, a):
        self.a = a

        self.maker_list = {
            MakeSimple.__name__: MakeSimple
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'NEW_BLEND_ATOM_MAKER': 'MakeSimple'
        }
        BlConfig().make_default_config(str(self), default_config)

    def get_maker(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'NEW_BLEND_ATOM_MAKER')

        maker = self.maker_list.get(str(id_or_name))
        if maker is not None:
            return maker(self.a)
        else:
            raise UserWarning('Maker not found.')

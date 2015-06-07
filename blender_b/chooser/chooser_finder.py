from blender_b.chooser.choose_all import ChooseAll
from blender_b.chooser.choose_in_blend_target import ChooseInBlendTarget
from blender_b.chooser.choose_in_sti_range import ChooseInSTIRange
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    def __init__(self, a):
        self.a = a

        self.chooser_list = {
            ChooseAll.__name__: ChooseAll,
            ChooseInBlendTarget.__name__: ChooseInBlendTarget,
            ChooseInSTIRange.__name__: ChooseInSTIRange
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'ATOMS_CHOOSER': 'ChooseAll'
        }
        BlConfig().make_default_config(str(self), default_config)

    def get_chooser(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'ATOMS_CHOOSER')

        chooser = self.chooser_list.get(str(id_or_name))
        if chooser is not None:
            return chooser(self.a)
        else:
            raise UserWarning('Chooser not found.')

from opencog_b.python.blending.chooser.choose_all import ChooseAll
from opencog_b.python.blending.chooser.choose_in_sti_range import ChooseInSTIRange
from opencog_b.python.blending.chooser.choose_null import ChooseNull
from opencog_b.python.blending.util.blend_config import BlendConfig

__author__ = 'DongMin Kim'


class ChooserFinder(object):
    def __init__(self, a):
        self.a = a

        self.choosers = {
            ChooseNull.__name__: ChooseNull,
            ChooseAll.__name__: ChooseAll,
            ChooseInSTIRange.__name__: ChooseInSTIRange
        }

    def __str__(self):
        return self.__class__.__name__

    def get_chooser(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlendConfig().get_str(self.a, "atoms-chooser")

        chooser = self.choosers.get(str(id_or_name))
        if chooser is not None:
            return chooser(self.a)
        else:
            raise UserWarning('Chooser not found.')

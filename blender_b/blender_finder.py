from blender_b.no_rule_blender import NoRuleBlender
from blender_b.rule_blender import RuleBlender
from util_b.general_util import BlConfig

__author__ = 'DongMin Kim'


class BlenderFinder(object):
    def __init__(self, a):
        self.a = a

        self.blender_list = {
            RuleBlender.__name__: RuleBlender,
            NoRuleBlender.__name__: NoRuleBlender
        }

        self.set_default_config()

    def __str__(self):
        return self.__class__.__name__

    def set_default_config(self):
        default_config = {
            'BLENDER': 'NoRuleBlender'
        }
        BlConfig().make_default_config(str(self), default_config)

    def get_blender(self, id_or_name=None):
        if id_or_name is None:
            id_or_name = BlConfig().get(str(self), 'BLENDER')

        blender = self.blender_list.get(str(id_or_name))
        if blender is not None:
            return blender(self.a)
        else:
            raise UserWarning('Blender not found.')

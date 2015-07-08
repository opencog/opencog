from opencog.logger import log
from blending.src.blender_factory import BlenderFactory

__author__ = 'DongMin Kim'

class ConceptualBlending:
    def __init__(self, a):
        self.a = a

        self.blender_factory = BlenderFactory(self.a)
        self.blender_inst = None

    def __blender_select(self, blender_name):
        self.blender_inst = self.blender_factory.get_blender(blender_name)
        if self.blender_inst is None:
            raise UserWarning(
                "Can't find " + blender_name + "\n" +
                "Available blenders: " + "\n" +
                str(self.blender_factory.blender_list)
            )

    def run(self, blender_name):
        log.warn("Start ConceptualBlending")
        self.__blender_select(blender_name)
        self.blender_inst.blend()

        if self.blender_inst.get_last_status() != 0:
            raise UserWarning('Error in blending class.')

        log.warn("Finish ConceptualBlending")

from blending.src.random_blender import RandomBlender

__author__ = 'DongMin Kim'

class ConceptualBlending:
    def __init__(self, a):
        self.a = a
        self.random_blending_inst = RandomBlender(self.a)

    def run(self):
        self.random_blending_inst.blend()

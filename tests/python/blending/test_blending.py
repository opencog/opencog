__author__ = 'DongMin Kim'

from opencog.atomspace import *

# Only run the unit tests if the required dependencies have been installed
# (see: https://github.com/opencog/opencog/issues/337)
try:
    __import__("nose.tools")
except ImportError:
    import unittest

    raise unittest.SkipTest(
        "ImportError exception: " +
        "Can't find Nose. " +
        "make sure the required dependencies are installed."
    )
else:
    from nose.tools import *

try:
    __import__("opencog.scheme_wrapper")
except ImportError:
    import unittest

    raise unittest.SkipTest(
        "ImportError exception: " +
        "Can't find Scheme wrapper for Python. " +
        "make sure the required dependencies are installed."
    )
else:
    from opencog.scheme_wrapper import *

try:
    __import__("blending.blend")
except ImportError:
    import unittest
    raise unittest.SkipTest(
        "ImportError exception: " +
        "Can't find Python Conceptual Blender. " +
        "make sure the required dependencies are installed."
    )
else:
    from blending.blend import ConceptualBlending


class TestConceptualBlending(object):
    """
    Unit tests for the OpenCog ConceptualBlending.
    """
    def __init__(self):
        self.a = None
        self.blender = None

    # noinspection PyPep8Naming
    def setUp(self):
        """This method is run once before _each_ test method is executed"""
        self.a = AtomSpace()
        self.blender = ConceptualBlending(self.a)

    # noinspection PyPep8Naming
    def tearDown(self):
        """This method is run once after _each_ test method is executed"""
        del self.a
        del self.blender

    def test_simple(self):
        car = self.a.add_node(types.ConceptNode, "car")
        man = self.a.add_node(types.ConceptNode, "man")
        self.a.set_av(car.h, 1)
        self.a.set_av(man.h, 1)

        result = self.blender.run()
        assert_equal(len(result), 1)

        blended_node = result[0]
        assert_true(
            str(blended_node.name) == "car-man" or
            str(blended_node.name) == "man-car"
        )

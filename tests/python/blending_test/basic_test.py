__author__ = 'DongMin Kim'

from opencog.atomspace import *
from test_conceptual_blending_base import TestConceptualBlendingBase

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
    # noinspection PyPackageRequirements
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

try:
    from blending.util.py_cog_execute import PyCogExecute
    PyCogExecute().load_scheme()
except (ImportError, RuntimeError):
    import unittest

    raise unittest.SkipTest(
        "Can't load Scheme." +
        "make sure the you installed atomspace to /usr/local/share/opencog."
    )


# noinspection PyArgumentList, PyTypeChecker, PyUnresolvedReferences
class TestBasicFunctions(TestConceptualBlendingBase):
    __test__ = True

    def test_simple(self):
        # Basic test uses very simple AtomSpace.
        del self.blender
        del self.a

        self.a = AtomSpace()

        self.blender = ConceptualBlending(self.a)

        car = self.a.add_node(types.ConceptNode, "car")
        man = self.a.add_node(types.ConceptNode, "man")
        self.a.set_av(car.h, 1)
        self.a.set_av(man.h, 1)

        # Test blender makes only one new blend node.
        result = self.blender.run()
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_true(
            str(blended_node.name) == "car-man" or
            str(blended_node.name) == "man-car"
        )

    def test_config_inheritance(self):
        try:
            __import__("blending.util.blending_config")
        except ImportError:
            import unittest

            raise unittest.SkipTest(
                "ImportError exception: " +
                "Can't load Blend Config. " +
                "make sure the required dependencies are installed."
            )
        else:
            from blending.util.blending_config import BlendConfig

        self.a.add_link(
            types.InheritanceLink,
            [
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "default-config")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseAll")
            ]
        )

        # Test config loads correctly.
        assert_equal(
            "ChooseNull",
            BlendConfig().get_str(
                self.a, "atoms-chooser", "default-config"
            )
        )

        # Test override config loads correctly.
        assert_equal(
            "ChooseAll",
            BlendConfig().get_str(
                self.a, "atoms-chooser", "my-config"
            )
        )

        # Test config loads parent's config correctly
        # if there not exists config.
        assert_equal(
            "DecideNull",
            BlendConfig().get_str(
                self.a, "blending-decider", "my-config"
            )
        )

    def test_config_update(self):
        try:
            __import__("blending.util.blending_config")
        except ImportError:
            import unittest

            raise unittest.SkipTest(
                "ImportError exception: " +
                "Can't load Blend Config. " +
                "make sure the required dependencies are installed."
            )
        else:
            from blending.util.blending_config import BlendConfig

        # Test config loads correctly.
        assert_equal(
            "ChooseNull",
            BlendConfig().get_str(
                self.a, "atoms-chooser", "default-config"
            )
        )

        BlendConfig().update(
            self.a,
            "atoms-chooser",
            "ChooseAll",
            "default-config"
        )

        # Test config updates correctly.
        assert_equal(
            "ChooseAll",
            BlendConfig().get_str(
                self.a, "atoms-chooser", "default-config"
            )
        )

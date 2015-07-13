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


# noinspection PyArgumentList, PyTypeChecker
class TestNewBlendAtomMaker(TestConceptualBlendingBase):
    """
    2.3 New Blend Atom Maker tests.
    """

    """
    2.3.1. MakeSimple tests.
    """
    __test__ = True

    def __default_make_simple(self):
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
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:new-blend-atom-maker"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "MakeSimple")
            ]
        )

    def test_make_simple(self):
        self.__default_make_simple()
        make_atom_prefix = "["
        make_atom_separator = "*"
        make_atom_postfix = "]"

        # Test blender makes new node with custom name.
        make_atom_prefix_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:make-atom-prefix"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, make_atom_prefix)
            ]
        )
        make_atom_separator_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:make-atom-separator"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, make_atom_separator)
            ]
        )
        make_atom_postfix_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:make-atom-postfix"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, make_atom_postfix)
            ]
        )

        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(make_atom_prefix_link)
        self.a.remove(make_atom_separator_link)
        self.a.remove(make_atom_postfix_link)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in(
            make_atom_prefix +
            "car" + make_atom_separator + "man" +
            make_atom_postfix,
            str(blended_node.name)
        )

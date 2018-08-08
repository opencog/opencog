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
class TestBlendingDecider(TestConceptualBlendingBase):
    """
    2.2. BlendingDecider tests.
    """

    """
    2.2.1. DecideNull tests.
    """
    __test__ = True

    def __default_decide_null(self):
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
                self.a.add_node(types.ConceptNode, "ChooseNull")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideNull")
            ]
        )

    def test_decide_null_without_focus_atoms(self):
        self.__default_decide_null()

        # Test blender not makes blend node if we don't give focus atoms.
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)

    def test_decide_null_with_focus_atoms(self):
        self.__default_decide_null()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            [self.sample_nodes["car"], self.sample_nodes["man"]],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("man", str(blended_node.name))
        assert_not_in("metal", str(blended_node.name))
        assert_not_in("move", str(blended_node.name))
        assert_not_in("vehicle", str(blended_node.name))
        assert_not_in("person", str(blended_node.name))

    """
    2.2.2. DecideRandom tests.
    """

    def __default_decide_random(self):
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
                self.a.add_node(types.ConceptNode, "ChooseInSTIRange")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideRandom")
            ]
        )

    def test_decide_random(self):
        self.__default_decide_random()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.ConceptNode),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        participated_focus_atoms_count = 0

        for focus_atom_name in map(
                lambda atom: atom.name, self.sample_nodes.values()
        ):
            if focus_atom_name in str(blended_node.name):
                participated_focus_atoms_count += 1

        assert_equal(participated_focus_atoms_count, 2)

    def test_decide_random_with_result_count_limit(self):
        self.__default_decide_random()
        decide_result_atoms_count = 3

        # Test blender limits count of result nodes correctly.
        decide_result_atoms_count_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:decide-result-atoms-count"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode,
                                str(decide_result_atoms_count)
                                )
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.ConceptNode),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        participated_focus_atoms_count = 0

        for focus_atom_name in map(
                lambda atom: atom.name, self.sample_nodes.values()
        ):
            if focus_atom_name in str(blended_node.name):
                participated_focus_atoms_count += 1

        assert_equal(participated_focus_atoms_count, decide_result_atoms_count)
        self.a.remove(decide_result_atoms_count_link)

    """
    2.2.3. DecideBestSTI tests.
    """

    def __default_decide_best_sti(self):
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
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )

    def test_decide_best_sti(self):
        self.__default_decide_best_sti()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.ConceptNode),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car-man", str(blended_node))

    def test_decide_best_sti_with_result_count_limit(self):
        self.__default_decide_best_sti()
        decide_result_atoms_count = 3

        # Test blender limits count of result nodes correctly.
        decide_result_atoms_count = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:decide-result-atoms-count"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode,
                                str(decide_result_atoms_count)
                                )
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.ConceptNode),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car-man-vehicle", str(blended_node))

        self.a.remove(decide_result_atoms_count)

    def test_decide_best_sti_with_min_sti_limit(self):
        self.__default_decide_best_sti()
        decide_sti_min = 20

        # Test blender limits sti value correctly.
        decide_sti_min_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:decide-sti-min"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, str(decide_sti_min))
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)
        self.a.remove(decide_sti_min_link)

    def test_decide_best_sti_with_max_sti_limit(self):
        self.__default_decide_best_sti()
        decide_sti_max = 10

        # Test blender limits sti value correctly.
        decide_sti_max_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:decide-sti-max"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, str(decide_sti_max))
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("metal-move", str(blended_node))
        self.a.remove(decide_sti_max_link)

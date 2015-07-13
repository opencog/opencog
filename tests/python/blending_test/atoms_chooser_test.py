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
class TestAtomsChooser(TestConceptualBlendingBase):
    """
    2.1. AtomsChooser tests.
    """

    """
    2.1.1. ChooseNull tests.
    """
    __test__ = True

    def __default_choose_null(self):
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

    def test_choose_null_without_focus_atoms(self):
        self.__default_choose_null()

        # Test blender not makes blend node if we don't give focus atoms.
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)

        # Test blender not makes blend node if we don't give focus atoms.
        result = self.blender.run(
            [],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)

    def test_choose_null_with_focus_atoms(self):
        self.__default_choose_null()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            [self.sample_nodes["car"], self.sample_nodes["man"],
             self.sample_nodes["metal"]],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("man", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_not_in("move", str(blended_node.name))
        assert_not_in("vehicle", str(blended_node.name))
        assert_not_in("person", str(blended_node.name))

    """
    2.1.2. ChooseAll tests.
    """

    def __default_choose_all(self):
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

    def test_choose_all(self):
        self.__default_choose_all()

        # Test blender doesn't explain if focus atoms was not given,
        # but find all atoms in AtomSpace.
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("person", str(blended_node.name))

        # Test blender doesn't explain if focus atoms was not given,
        # but find all atoms in AtomSpace.
        result = self.blender.run(
            [],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("person", str(blended_node.name))

        # Test blender makes only one new blend node.
        result = self.blender.run(
            [self.sample_nodes["car"], self.sample_nodes["vehicle"]],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_not_in("man", str(blended_node.name))
        assert_not_in("metal", str(blended_node.name))
        assert_not_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_not_in("person", str(blended_node.name))

    def test_choose_all_with_type_limit(self):
        self.__default_choose_all()

        # Test blender limits node type correctly.
        choose_atom_type_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-atom-type"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "PredicateNode")
            ]
        )
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)
        self.a.remove(choose_atom_type_link)

        # Test blender limits node type correctly.
        choose_atom_type_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-atom-type"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ConceptNode")
            ]
        )
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("person", str(blended_node.name))
        self.a.remove(choose_atom_type_link)

    def test_choose_all_with_count_limit(self):
        self.__default_choose_all()

        # Test blender checks node count correctly.
        choose_least_count_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-least-count"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "1000")
            ]
        )
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)
        self.a.remove(choose_least_count_link)

        # Test blender checks node count correctly.
        choose_least_count_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-least-count"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "2")
            ]
        )
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)
        self.a.remove(choose_least_count_link)

    """
    2.1.3. ChooseInSTIRange tests.
    """

    def __default_choose_in_sti_range(self):
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

    def test_choose_in_sti_range_without_focus_atoms(self):
        self.__default_choose_in_sti_range()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            None,
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)

    def test_choose_in_sti_range_with_focus_atoms(self):
        self.__default_choose_in_sti_range()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("person", str(blended_node.name))

    def test_choose_in_sti_range_with_type_limit(self):
        self.__default_choose_in_sti_range()

        # Test blender limits node type correctly.
        choose_atom_type_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-atom-type"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "PredicateNode")
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)
        self.a.remove(choose_atom_type_link)

        # Test blender limits node type correctly.
        choose_atom_type_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-atom-type"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ConceptNode")
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_in("car", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("metal", str(blended_node.name))
        assert_in("move", str(blended_node.name))
        assert_in("vehicle", str(blended_node.name))
        assert_in("person", str(blended_node.name))
        self.a.remove(choose_atom_type_link)

    def test_choose_in_sti_range_with_count_limit(self):
        self.__default_choose_in_sti_range()

        # Test blender limits node count correctly.
        choose_least_count_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-least-count"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "1000")
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 0)
        self.a.remove(choose_least_count_link)

        # Test blender limits node count correctly.
        choose_least_count_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-least-count"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "2")
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)
        self.a.remove(choose_least_count_link)

    def test_choose_in_sti_range_with_min_sti_limit(self):
        self.__default_choose_in_sti_range()
        choose_sti_min = 15

        # Test blender limits sti value correctly.
        choose_sti_min_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-sti-min"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, str(choose_sti_min))
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )

        # Test blender limits sti value correctly.
        blended_node = result[0]
        assert_true(
            str(blended_node.name) == "car-man" or
            str(blended_node.name) == "man-car"
        )
        self.a.remove(choose_sti_min_link)

    def test_choose_in_sti_range_with_max_sti_limit(self):
        self.__default_choose_in_sti_range()
        choose_sti_max = 5

        # Test blender limits sti value correctly.
        choose_sti_max_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:choose-sti-max"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, str(choose_sti_max))
            ]
        )
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        # Test blender makes new blend node correctly.
        blended_node = result[0]
        assert_true(
            str(blended_node.name) == "metal-move" or
            str(blended_node.name) == "move-metal"
        )
        self.a.remove(choose_sti_max_link)

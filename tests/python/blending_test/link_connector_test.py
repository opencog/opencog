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
class TestLinkConnector(TestConceptualBlendingBase):
    """
    2.4 LinkConnector tests.
    """

    """
    2.4.1. ConnectSimple tests.
    """
    __test__ = True

    def __default_connect_simple(self):
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
                self.a.add_node(types.SchemaNode, "BLEND:link-connector"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ConnectSimple")
            ]
        )

    def test_connect_simple(self):
        self.__default_connect_simple()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                # (0.9 * 0.8 + 0.7 * 0.9) / (0.8 + 0.9) = 0.794117
                assert_less_equal(link.tv.mean, 0.81)
                assert_greater_equal(link.tv.mean, 0.79)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                # (0.9 * 0.8 + 0.1 * 0.9) / (0.8 + 0.9) = 0.476471
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                # (0.1 * 0.8 + 0.8 * 0.9) / (0.8 + 0.9) = 0.470588
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)

    """
    2.4.2. ConnectConflictRandom tests.
    """

    def __default_connect_conflict_random(self):
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
                self.a.add_node(types.SchemaNode, "BLEND:link-connector"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ConnectConflictRandom")
            ]
        )

    def test_connect_conflict_random(self):
        self.__default_connect_conflict_random()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                # (0.9 * 0.8 + 0.7 * 0.9) / (0.8 + 0.9) = 0.794117
                assert_less_equal(link.tv.mean, 0.81)
                assert_greater_equal(link.tv.mean, 0.79)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.9, 0.8)
                    assert_equal(link.tv.mean, 0.9)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.1, 0.9)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.9)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.1, 0.8)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.8, 0.9)
                    assert_equal(link.tv.mean, 0.8)
                    assert_equal(link.tv.confidence, 0.9)

    def test_connect_conflict_random_with_strength_diff_limit(self):
        self.__default_connect_conflict_random()

        # Test blender thinks links are conflict
        # if they have difference value above 0.9 of strength.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.9")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        # There not exists conflict links
        # because threshold of strength value was set in 0.9.
        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                # (0.9 * 0.8 + 0.7 * 0.9) / (0.8 + 0.9) = 0.794117
                assert_less_equal(link.tv.mean, 0.81)
                assert_greater_equal(link.tv.mean, 0.79)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                # (0.9 * 0.8 + 0.1 * 0.9) / (0.8 + 0.9) = 0.476471
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                # (0.1 * 0.8 + 0.8 * 0.9) / (0.8 + 0.9) = 0.470588
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)

        # Test blender thinks links are conflict
        # if they have difference value above 0.01 of strength.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.01")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        # There exists 3 conflict links
        # because threshold of strength value was set in 0.01.
        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.9, 0.8)
                    assert_equal(link.tv.mean, 0.9)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.7, 0.9)
                    assert_equal(link.tv.mean, 0.7)
                    assert_equal(link.tv.confidence, 0.9)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.9, 0.8)
                    assert_equal(link.tv.mean, 0.9)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.1, 0.9)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.9)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.1, 0.8)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.8, 0.9)
                    assert_equal(link.tv.mean, 0.8)
                    assert_equal(link.tv.confidence, 0.9)

    def test_connect_conflict_random_with_confidence_above_limit(self):
        self.__default_connect_conflict_random()

        # Test blender thinks links are conflict
        # if they have confidence value above 0.9 both.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-confidence-above-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.9")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        # There not exists conflict links
        # because threshold of confidence value was set in 0.9.
        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                # (0.9 * 0.8 + 0.7 * 0.9) / (0.8 + 0.9) = 0.794117
                assert_less_equal(link.tv.mean, 0.81)
                assert_greater_equal(link.tv.mean, 0.79)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                # (0.9 * 0.8 + 0.1 * 0.9) / (0.8 + 0.9) = 0.476471
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                # (0.1 * 0.8 + 0.8 * 0.9) / (0.8 + 0.9) = 0.470588
                assert_less_equal(link.tv.mean, 0.48)
                assert_greater_equal(link.tv.mean, 0.46)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)

        # Test blender thinks links are conflict
        # if they have confidence value above 0.4 both.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-confidence-above-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.4")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        # There exists 2 conflict links
        # because threshold of strength value was set in 0.4.
        for link in blended_node.incoming:
            dst_nodes_name = map(lambda atom: atom.name, link.out)
            if "metal" in dst_nodes_name:
                # A. Not duplicated link.
                assert_almost_equal(link.tv.mean, 0.6)
                assert_almost_equal(link.tv.confidence, 0.8)
            elif "move" in dst_nodes_name:
                # B. Duplicated, not conflicted link.
                # (0.9 * 0.8 + 0.7 * 0.9) / (0.8 + 0.9) = 0.794117
                assert_less_equal(link.tv.mean, 0.81)
                assert_greater_equal(link.tv.mean, 0.79)
                # (0.8 + 0.9) / 2 = 0.85
                assert_less_equal(link.tv.confidence, 0.86)
                assert_greater_equal(link.tv.confidence, 0.84)
            elif "vehicle" in dst_nodes_name:
                # C.1 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.9, 0.8)
                    assert_equal(link.tv.mean, 0.9)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.1, 0.9)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.9)
            elif "person" in dst_nodes_name:
                # C.2 Duplicated, conflicted link.
                if "car" in dst_nodes_name:
                    # Randomly selected. - car's link - (0.1, 0.8)
                    assert_equal(link.tv.mean, 0.1)
                    assert_equal(link.tv.confidence, 0.8)
                elif "man" in dst_nodes_name:
                    # Randomly selected. - man's link - (0.8, 0.9)
                    assert_equal(link.tv.mean, 0.8)
                    assert_equal(link.tv.confidence, 0.9)

    """
    2.4.3. ConnectConflictAllViable tests.
    """

    def __default_connect_conflict_all_viable(self):
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
                self.a.add_node(types.SchemaNode, "BLEND:link-connector"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ConnectConflictAllViable")
            ]
        )

    def test_connect_conflict_all_viable(self):
        self.__default_connect_conflict_all_viable()

        # Test blender makes all viable new blend nodes = 2^2 = 4.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 4)

    def test_connect_conflict_all_viable_with_strength_diff_limit(self):
        self.__default_connect_conflict_all_viable()

        # Test blender thinks links are conflict
        # if they have difference value above 0.9 of strength.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.9")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender thinks links are conflict
        # if they have difference value above 0.01 of strength.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.01")
            ]
        )

        # Test blender makes all viable new blend nodes = 2^3 = 8.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 8)

    def test_connect_conflict_all_viable_with_confidence_above_limit(self):
        self.__default_connect_conflict_all_viable()

        # Test blender thinks links are conflict
        # if they have confidence value above 0.9 both.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.9")
            ]
        )

        # Test blender makes only one new blend node.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 1)

        # Test blender thinks links are conflict
        # if they have confidence value above 0.4 both.
        connect_strength_diff_limit_link = self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode,
                                "BLEND:connect-strength-diff-limit"
                                ),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "0.4")
            ]
        )

        # Test blender makes all viable new blend nodes = 2^2 = 4.
        result = self.blender.run(
            self.a.get_atoms_by_type(types.Node),
            self.a.add_node(types.ConceptNode, "my-config")
        )
        self.a.remove(connect_strength_diff_limit_link)
        assert_equal(len(result), 4)

    """
    2.4.4. ConnectConflictInteractionInformation tests.
    """

    def __default_connect_conflict_interaction_information(self):
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
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:link-connector"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode,
                                "ConnectConflictInteractionInformation"
                                )
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:connect-check-type"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "SimilarityLink")
            ]
        )

    def test_connect_conflict_interaction_information(self):
        self.__default_connect_conflict_interaction_information()

        # Test blender makes only one new blend node.
        result = self.blender.run(
            [
                self.a.add_node(types.ConceptNode, "car"),
                self.a.add_node(types.ConceptNode, "man")
            ],
            self.a.add_node(types.ConceptNode, "my-config")
        )
        assert_equal(len(result), 1)

        # Test blender makes new blend node correctly.
        blended_node = result[0]

        # In this test case, interaction information algorithm says
        # a link set includes 'move, metal, person, vehicle' is most surprising.
        #
        # [Selected]: move, metal, person, vehicle, : 0.643179893494
        # move, metal, person, : 0.488187074661
        # move, metal, vehicle, : 0.4154522717
        # move, metal, : 0.0
        dst_nodes_name = list()
        for link in blended_node.incoming:
            dst_nodes_name.extend(map(lambda atom: atom.name, link.out))
        assert_in("move", dst_nodes_name)
        assert_in("metal", dst_nodes_name)
        assert_in("person", dst_nodes_name)
        assert_in("vehicle", dst_nodes_name)

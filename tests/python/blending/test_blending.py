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


# noinspection PyArgumentList, PyTypeChecker
class TestConceptualBlendingBase(object):
    """
    Unit tests for the OpenCog ConceptualBlending.

    Attributes:
        :type a: opencog.atomspace_details.AtomSpace
    """
    __test__ = False

    def __init__(self):
        self.a = AtomSpace()
        self.blender = None
        self.sample_nodes = dict()

    # noinspection PyPep8Naming
    def setUp(self):
        """This method is run once before _each_ test method is executed"""
        self.blender = ConceptualBlending(self.a)

        """
        Make test nodes.
        """
        # Nodes will be blended:
        self.sample_nodes["car"] = self.a.add_node(types.ConceptNode, "car")
        self.sample_nodes["man"] = self.a.add_node(types.ConceptNode, "man")

        # A. Car is metal. (Not duplicated)
        self.sample_nodes["metal"] = self.a.add_node(types.ConceptNode, "metal")

        # B. Car moves, man moves. (Duplicated, not conflicted)
        self.sample_nodes["move"] = self.a.add_node(types.ConceptNode, "move")

        # C.1. Car is vehicle, man is not vehicle. (Duplicated and conflicted)
        # C.2. Car is not person, man is person. (Duplicated and conflicted)
        self.sample_nodes["vehicle"] = self.a.add_node(
            types.ConceptNode, "vehicle"
        )
        self.sample_nodes["person"] = self.a.add_node(
            types.ConceptNode, "person"
        )

        """
        Give some stimulates.
        """
        self.a.set_av(self.sample_nodes["car"].h, 19)
        self.a.set_av(self.sample_nodes["man"].h, 18)

        self.a.set_av(self.sample_nodes["metal"].h, 2)
        self.a.set_av(self.sample_nodes["move"].h, 1)

        self.a.set_av(self.sample_nodes["vehicle"].h, 13)
        self.a.set_av(self.sample_nodes["person"].h, 12)

        """
        Make test links.
        """
        # A. Not duplicated link.
        l1 = self.a.add_link(
            types.MemberLink,
            [
                self.sample_nodes["car"],
                self.sample_nodes["metal"]
            ]
        )
        self.a.set_tv(l1.h, TruthValue(0.6, 0.8))

        # B. Duplicated, not conflicted link.
        l2 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["car"],
                self.sample_nodes["move"]
            ]
        )
        l3 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["man"],
                self.sample_nodes["move"]
            ]
        )
        self.a.set_tv(l2.h, TruthValue(0.9, 0.8))
        self.a.set_tv(l3.h, TruthValue(0.7, 0.9))

        # C.1 Duplicated, conflicted link.
        l4 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["car"],
                self.sample_nodes["vehicle"]
            ]
        )
        l5 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["man"],
                self.sample_nodes["vehicle"]
            ]
        )
        self.a.set_tv(l4.h, TruthValue(0.9, 0.8))
        self.a.set_tv(l5.h, TruthValue(0.1, 0.9))

        # C.2 Duplicated, conflicted link.
        l6 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["car"],
                self.sample_nodes["person"]
            ]
        )
        l7 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["man"],
                self.sample_nodes["person"]
            ]
        )
        self.a.set_tv(l6.h, TruthValue(0.1, 0.8))
        self.a.set_tv(l7.h, TruthValue(0.8, 0.9))

        """
        Make default configs.
        """
        self.a.add_link(
            types.InheritanceLink,
            [
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "BLEND")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "ChooseNull")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "DecideNull")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:new-blend-atom-maker"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "MakeSimple")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:link-connector"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "ConnectSimple")
            ]
        )

    # noinspection PyPep8Naming
    def tearDown(self):
        """This method is run once after _each_ test method is executed"""
        self.a.clear()
        del self.sample_nodes
        del self.blender


"""
1. Test basic functions.
"""


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
            __import__("blending.util.blend_config")
        except ImportError:
            import unittest

            raise unittest.SkipTest(
                "ImportError exception: " +
                "Can't load Blend Config. " +
                "make sure the required dependencies are installed."
            )
        else:
            from blending.util.blend_config import BlendConfig

        self.a.add_link(
            types.InheritanceLink,
            [
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "default-config")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            __import__("blending.util.blend_config")
        except ImportError:
            import unittest

            raise unittest.SkipTest(
                "ImportError exception: " +
                "Can't load Blend Config. " +
                "make sure the required dependencies are installed."
            )
        else:
            from blending.util.blend_config import BlendConfig

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


"""
2. Test with configs.
"""


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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseNull")
            ]
        )
        self.a.add_link(
            types.ListLink,
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

        assert_not_equal(
            self.blender.decider.last_status,
            self.blender.decider.Status.SUCCESS_DECIDE
        )

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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseInSTIRange")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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

        assert_not_equal(
            self.blender.decider.last_status,
            self.blender.decider.Status.SUCCESS_DECIDE
        )
        self.a.remove(decide_sti_min_link)

    def test_decide_best_sti_with_max_sti_limit(self):
        self.__default_decide_best_sti()
        decide_sti_max = 10

        # Test blender limits sti value correctly.
        decide_sti_max_link = self.a.add_link(
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseAll")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:make-atom-prefix"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, make_atom_prefix)
            ]
        )
        make_atom_separator_link = self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:make-atom-separator"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, make_atom_separator)
            ]
        )
        make_atom_postfix_link = self.a.add_link(
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseAll")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseAll")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "ChooseAll")
            ]
        )
        self.a.add_link(
            types.ListLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "my-config"),
                self.a.add_node(types.ConceptNode, "DecideBestSTI")
            ]
        )
        self.a.add_link(
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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
            types.ListLink,
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

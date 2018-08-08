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

        # D. Make inherit relation to test interaction information algorithm.
        self.sample_nodes["bumblebee"] = self.a.add_node(
            types.ConceptNode, "bumblebee"
        )
        self.sample_nodes["suv"] = self.a.add_node(
            types.ConceptNode, "suv"
        )
        self.sample_nodes["rickshaw"] = self.a.add_node(
            types.ConceptNode, "rickshaw"
        )
        self.sample_nodes["programmer"] = self.a.add_node(
            types.ConceptNode, "programmer"
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

        # D. Make inherit relation to test interaction information algorithm.
        l8 = self.a.add_link(
            types.InheritanceLink,
            [
                self.sample_nodes["bumblebee"],
                self.sample_nodes["car"]
            ]
        )
        l9 = self.a.add_link(
            types.InheritanceLink,
            [
                self.sample_nodes["suv"],
                self.sample_nodes["car"]
            ]
        )
        l10 = self.a.add_link(
            types.InheritanceLink,
            [
                self.sample_nodes["rickshaw"],
                self.sample_nodes["man"]
            ]
        )
        l11 = self.a.add_link(
            types.InheritanceLink,
            [
                self.sample_nodes["programmer"],
                self.sample_nodes["man"]
            ]
        )

        # E Give properties to specific entity.
        l12 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["bumblebee"],
                self.sample_nodes["vehicle"]
            ]
        )
        l13 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["bumblebee"],
                self.sample_nodes["person"]
            ]
        )
        self.a.set_tv(l12.h, TruthValue(0.8, 0.9))
        self.a.set_tv(l13.h, TruthValue(0.6, 0.9))

        l14 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["suv"],
                self.sample_nodes["vehicle"]
            ]
        )
        l15 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["suv"],
                self.sample_nodes["person"]
            ]
        )
        self.a.set_tv(l14.h, TruthValue(0.9, 0.9))
        self.a.set_tv(l15.h, TruthValue(0.2, 0.9))

        l16 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["rickshaw"],
                self.sample_nodes["vehicle"]
            ]
        )
        l17 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["rickshaw"],
                self.sample_nodes["person"]
            ]
        )
        self.a.set_tv(l16.h, TruthValue(0.4, 0.9))
        self.a.set_tv(l17.h, TruthValue(0.9, 0.9))

        l18 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["programmer"],
                self.sample_nodes["vehicle"]
            ]
        )
        l19 = self.a.add_link(
            types.SimilarityLink,
            [
                self.sample_nodes["programmer"],
                self.sample_nodes["person"]
            ]
        )
        self.a.set_tv(l18.h, TruthValue(0.1, 0.9))
        self.a.set_tv(l19.h, TruthValue(0.8, 0.9))

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
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:atoms-chooser"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "ChooseNull")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:blending-decider"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "DecideNull")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
            [
                self.a.add_node(types.SchemaNode, "BLEND:new-blend-atom-maker"),
                self.a.add_node(types.ConceptNode, "default-config"),
                self.a.add_node(types.ConceptNode, "MakeSimple")
            ]
        )
        self.a.add_link(
            types.ExecutionLink,
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

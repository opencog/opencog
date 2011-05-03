from unittest import TestCase

import opencog
from opencog import AtomSpace, TruthValue, types
from atom import Atom 

class AtomSpaceTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()

    def tearDown(self):
        pass #del self.space

    def test_add_node(self):

        h1 = self.space.add_node(types.Node,"test")
        self.assertFalse(h1.is_undefined())
        # duplicates resolve to same handle
        h2 = self.space.add_node(types.Node,"test")
        self.assertEquals(h1,h2)

        # fails when adding with a link type
        h1 = self.space.add_node(types.Link,"test")
        self.assertEquals(h1,None)

        # test adding with a truthvalue
        h3 = self.space.add_node(types.Node,"test_w_tv",TruthValue(0.5,100))
        self.assertEquals(self.space.size(),2)

        # test adding with prefixed node
        h1 = self.space.add_node(types.Node,"test",prefixed=True)
        h2 = self.space.add_node(types.Node,"test",prefixed=True)
        self.assertNotEqual(h1,h2)
        self.assertEquals(self.space.size(),4)

        h3 = self.space.add_node(types.Node,"test",TruthValue(0.5,100),prefixed=True)
        self.assertNotEqual(h1,h3)
        self.assertEquals(self.space.size(),5)

        # tests with bad parameters
        # test with not a proper truthvalue
        self.assertRaises(TypeError,self.space.add_node,types.Node,"test",0,True)
        # test with bad type
        self.assertRaises(TypeError,self.space.add_node,"ConceptNode","test",TruthValue(0.5,100))

    def test_add_link(self):
        h1 = self.space.add_node(types.Node,"test1")
        h2 = self.space.add_node(types.Node,"test2")
        l1 = self.space.add_link(types.Link,[h1,h2])
        self.assertTrue(l1 is not None)
        l2 = self.space.add_link(types.Link,[h1,h2])
        self.assertTrue(l2 is not None)
        self.assertTrue(l2 == l1)

        h3 = self.space.add_node(types.Node,"test3")
        l3 = self.space.add_link(types.Link,[h1,h3],TruthValue(0.5,100))
        self.assertTrue(l3 is not None)

        # fails when adding with a node type
        h1 = self.space.add_link(1,[h1,h3])
        self.assertEquals(h1,None)

    def test_is_valid(self):
        h1 = self.space.add_node(types.Node,"test1")
        # check with Handle object
        self.assertTrue(self.space.is_valid(h1))
        # check with raw UUID
        self.assertTrue(self.space.is_valid(h1.value()))
        # check with bad UUID
        self.assertFalse(self.space.is_valid(2919))
        # check with bad type
        self.assertRaises(TypeError,self.space.is_valid,"test")

    def test_truth_value(self):
        # check attributes come back as assigned
        tv = TruthValue(0.5, 100)
        self.assertEqual(tv.mean,0.5)
        self.assertEqual(tv.count,100)
        # test confidence
        self.assertAlmostEqual(tv.confidence,0.1111,places=4)
        # test string representation
        self.assertEqual(str(tv),"[0.500000,100.000000=0.111111]")

        # check equality
        tv2 = TruthValue(0.5, 100)
        tv3 = TruthValue(0.6, 100)
        self.assertTrue(tv == tv2)
        self.assertFalse(tv == tv3)

    def test_get_by_name_and_type(self):
        h1 = self.space.add_node(types.Node,"test")
        h2 = self.space.add_node(types.ConceptNode,"test")
        h3 = self.space.add_node(types.PredicateNode,"test")

        # test recursive subtypes
        result = self.space.get_atoms_by_name(types.Node,"test")
        self.assertTrue(h1 in result)
        self.assertTrue(h2 in result)
        self.assertTrue(h3 in result)

        # test non-recursive subtype
        result = self.space.get_atoms_by_name(types.Node,"test",subtype=False)
        self.assertTrue(h1 in result)
        self.assertTrue(h2 not in result)
        self.assertTrue(h3 not in result)

        # test empty
        result = self.space.get_atoms_by_name(types.SemeNode,"test",subtype=False)
        self.assertEqual(len(result),0)

    def test_get_by_type(self):
        h1 = self.space.add_node(types.Node,"test1")
        h2 = self.space.add_node(types.ConceptNode,"test2")
        h3 = self.space.add_node(types.PredicateNode,"test3")

        # test recursive subtypes
        result = self.space.get_atoms_by_type(types.Node)
        self.assertTrue(h1 in result)
        self.assertTrue(h2 in result)
        self.assertTrue(h3 in result)

        # links
        l1 = self.space.add_link(types.InheritanceLink,[h1,h2])
        result = self.space.get_atoms_by_type(types.Link)
        self.assertTrue(l1 in result)
        
        # test non-recursive subtype
        result = self.space.get_atoms_by_type(types.Node,subtype=False)
        self.assertTrue(h1 in result)
        self.assertTrue(h2 not in result)
        self.assertTrue(h3 not in result)

        # test empty
        result = self.space.get_atoms_by_type(types.SemeNode,subtype=False)
        self.assertEqual(len(result),0)

    def test_get_by_target_type(self):
        h1 = self.space.add_node(types.Node,"test1")
        h2 = self.space.add_node(types.ConceptNode,"test2")
        h3 = self.space.add_node(types.PredicateNode,"test3")

        # test it doesn't apply to Nodes
        result = self.space.get_atoms_by_target_type(types.Node,types.Node)
        self.assertTrue(h1 not in result)

        # links
        l1 = self.space.add_link(types.InheritanceLink,[h1,h2])
        result = self.space.get_atoms_by_target_type(types.Link,types.ConceptNode,target_subtype=False)
        self.assertTrue(l1 in result)
        
        # test recursive target subtype
        result = self.space.get_atoms_by_target_type(types.Link,types.Node,target_subtype=True)
        self.assertTrue(l1 in result)

    def test_get_by_target_handle(self):
        h1 = self.space.add_node(types.Node,"test1")
        h2 = self.space.add_node(types.ConceptNode,"test2")
        h3 = self.space.add_node(types.PredicateNode,"test3")

        # test it doesn't apply to Nodes
        result = self.space.get_atoms_by_target_handle(types.Node,h1)
        self.assertTrue(h1 not in result)

        # links
        l1 = self.space.add_link(types.InheritanceLink,[h1,h2])
        result = self.space.get_atoms_by_target_handle(types.Link,h1)
        self.assertTrue(l1 in result)
        result = self.space.get_atoms_by_target_handle(types.Link,h3)
        self.assertTrue(l1 not in result)

class AtomTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()

    def test_creation(self):
        h1 = self.space.add_node(types.Node,"test1")
        a = Atom(h1, self.space)
        self.assertEqual(a.name,"test1")
        self.assertEqual(a.tv,TruthValue(0.0,0.0))

    def test_w_truthvalue(self):
        tv = TruthValue(0.5, 100)
        h2 = self.space.add_node(types.Node,"test2",tv)
        a = Atom(h2, self.space)
        self.assertEqual(a.tv,tv)

        # test set tv
        a.tv = TruthValue(0.1,10)
        self.assertEqual(a.tv,TruthValue(0.1,10))
        
    def test_w_attention_value(self):
        h = self.space.add_node(types.Node,"test2")
        a = Atom(h, self.space)

        self.assertEqual(a.av,{'lti': 0, 'sti': 0, 'vlti': False})

        # test set av
        a.av = { "sti": 10, "lti": 1, "vlti": True }
        self.assertEqual(a.av,{'sti': 10, 'lti': 1, 'vlti': True})

    def test_out(self):
        # test get out
        h1 = self.space.add_node(types.Node,"test2")
        a = Atom(h1, self.space)

        self.assertEqual(a.out, [])

        tv = TruthValue(0.5, 100)
        h2 = self.space.add_node(types.Node,"test3",tv)

        l1 = self.space.add_link(types.Link,[h1,h2])
        l = Atom(l1, self.space)
        self.assertEqual(l.out, [h1,h2])

        # ensure out is considered immutable
        self.assertRaises(AttributeError, setattr, l,"out",[h1])

    def test_is_source(self):
        # any out item is a source for unordered links
        # only the fist item is a source of ordered links
        h1 = self.space.add_node(types.Node,"test1")
        h2 = self.space.add_node(types.Node,"test2")

        l1 = self.space.add_link(types.OrderedLink,[h1,h2])
        l_ordered = Atom(l1, self.space)
        l2 = self.space.add_link(types.UnorderedLink,[h1,h2])
        l_unordered = Atom(l2, self.space)

        self.assertEqual(l_ordered.is_source(h1), True)
        self.assertEqual(l_ordered.is_source(h2), False)

        self.assertEqual(l_unordered.is_source(h1), True)
        self.assertEqual(l_unordered.is_source(h2), True)

    def test_type(self):
        # test get out
        h1 = self.space.add_node(types.Node,"test2")
        a = Atom(h1, self.space)

        self.assertEqual(a.type, 1)

        h2 = self.space.add_node(types.Node,"test3")
        l1 = self.space.add_link(types.Link,[h1,h2])
        l = Atom(l1, self.space)
        self.assertEqual(l.type, 2)

        # ensure type is considered immutable
        self.assertRaises(AttributeError, setattr, l,"type",5)
        self.assertRaises(AttributeError, setattr, a,"type",5)


    def test_strings(self):
        # set up a link and atoms
        tv = TruthValue(0.5, 100)
        h1 = self.space.add_node(types.Node,"test1",tv)

        h2 = self.space.add_node(types.Node,"test2")
        a = Atom(h2, self.space)
        a.av = { "sti": 10, "lti": 1, "vlti": True }
        a.tv = TruthValue(0.1,10)

        l1 = self.space.add_link(types.Link,[h1,h2])
        l = Atom(l1, self.space)

        # test string representation
        self.assertEqual(str(a),"node[Node:test2]")
        self.assertEqual(a.long_string(),
                "node[Node:test2] av:(10,1) tv:([0.100000,10.000000=0.012346])")
        self.assertEqual(str(l),"[Link <test1,test2> 0.000000 0.000000]")
        self.assertEqual(l.long_string(),
                "link[Link sti:(0,0) tv:([0.000000,0.000000=0.000000]) <[Node test1],[Node test2]>]")

class TypeTest(TestCase):

    def test_is_a(self):
        self.assertTrue(opencog.is_a(types.ConceptNode,types.Node))
        self.assertTrue(opencog.is_a(types.ConceptNode,types.Atom))

        self.assertTrue(opencog.is_a(types.ListLink,types.Link))
        self.assertTrue(opencog.is_a(types.ListLink,types.Atom))

        self.assertFalse(opencog.is_a(types.Link,types.Node))

    def test_get_type(self):
        self.assertEqual(opencog.get_type("ConceptNode"), types.ConceptNode)
        self.assertEqual(opencog.get_type(""), types.NO_TYPE)
        self.assertRaises(TypeError,opencog.get_type,1)

    def test_get_type_name(self):
        self.assertEqual(opencog.get_type_name(types.Node), "Node")
        self.assertEqual(opencog.get_type_name(2231), "")
        self.assertEqual(opencog.get_type_name(types.NO_TYPE), "")




from unittest import TestCase

from opencog import AtomSpace, TruthValue, Atom

class AtomSpaceTest(TestCase):

    def setUp(self):
        self.space = AtomSpace()

    def tearDown(self):
        pass

    def test_add_node(self):
        h1 = self.space.add_node(1,"test")
        self.assertFalse(h1.is_undefined())
        # duplicates resolve to same handle
        h2 = self.space.add_node(1,"test")
        self.assertEquals(h1,h2)

        # fails when adding with a link type
        h1 = self.space.add_node(2,"test")
        self.assertEquals(h1,None)

        # test adding with a truthvalue
        h3 = self.space.add_node(1,"test_w_tv",TruthValue(0.5,100))
        self.assertEquals(self.space.size(),2)

        # test adding with prefixed node
        h1 = self.space.add_node(1,"test",prefixed=True)
        h2 = self.space.add_node(1,"test",prefixed=True)
        self.assertNotEqual(h1,h2)
        self.assertEquals(self.space.size(),4)

        h3 = self.space.add_node(1,"test",TruthValue(0.5,100),prefixed=True)
        self.assertNotEqual(h1,h3)
        self.assertEquals(self.space.size(),5)

        # tests with bad parameters
        # test with not a proper truthvalue
        self.assertRaises(TypeError,self.space.add_node,1,"test",0,True)
        # test with bad type
        self.assertRaises(TypeError,self.space.add_node,"ConceptNode","test",TruthValue(0.5,100))

    def test_add_link(self):
        h1 = self.space.add_node(1,"test1")
        h2 = self.space.add_node(1,"test2")
        l1 = self.space.add_link(2,[h1,h2])
        self.assertTrue(l1 is not None)
        l2 = self.space.add_link(2,[h1,h2])
        self.assertTrue(l2 is not None)
        self.assertTrue(l2 == l1)

        h3 = self.space.add_node(1,"test3")
        l3 = self.space.add_link(2,[h1,h3],TruthValue(0.5,100))
        self.assertTrue(l3 is not None)

        # fails when adding with a link type
        h1 = self.space.add_link(1,[h1,h3])
        self.assertEquals(h1,None)

    def test_is_valid(self):
        h1 = self.space.add_node(1,"test1")
        # check with Handle object
        self.assertTrue(self.space.is_valid(h1))
        # check with raw UUID
        self.assertTrue(self.space.is_valid(h1.value()))
        # check with bad UUID
        self.assertFalse(self.space.is_valid(2919))
        # check with bad type
        self.assertRaises(TypeError,self.space.is_valid,"test")

    def test_atom(self):
        h1 = self.space.add_node(1,"test1")
        a = Atom(h1, self.space)
        self.assertEqual(a.name,"test1")
        self.assertEqual(a.tv,TruthValue(0.0,0.0))

        tv = TruthValue(0.5, 100)
        h2 = self.space.add_node(1,"test2",tv)
        a = Atom(h2, self.space)
        self.assertEqual(a.tv,tv)

        self.assertEqual(a.av,{'lti': 0, 'sti': 0, 'vlti': False})

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
        
        






__author__ = 'kdm'

import os.path; import sys

# To avoid unresolved reference complain in PyCharm 4.0.6
from opencog.atomspace import AtomSpace, TruthValue, types
from opencog.bindlink import bindlink
from opencog.type_constructors import ConceptNode, UnorderedLink
from opencog.utilities import initialize_opencog
from opencog.logger import log
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__

sys.path.append(os.path.expanduser("~/opencog/opencog/python"))
import web.api.restapi


# Make several test cases for debug.
class TestCaseMaker:
    def __init__(self, atomspace):
        self.a = atomspace
        self.atom_list_for_debug = []
        self.link_list_for_debug = []

    def _make_test_case(self):
        # Paul & Sally example in the book 'The Way We Think'
        a1 = ConceptNode("Paul", TruthValue(0.9, 0.8))
        a2 = ConceptNode("Sally", TruthValue(0.9, 0.8))
        l1 = UnorderedLink([a1, a2], TruthValue(1.0, 0.8))
        self.atom_list_for_debug.extend([a1, a2])
        self.link_list_for_debug.append(l1)

    def make(self):
        self._make_test_case()


# Note: Divided to standalone class because I'll remove
class RESTAPILoader:
    def __init__(self, atomspace):
        self.a = atomspace
        self.restapi = web.api.restapi.Start()

    def run(self):
        self.restapi.run("", self.a)


# Class for dongmin's practice & experiment & test
class ToyClass:
    def __init__(self, atomspace):
        self.a = atomspace

    def foo(self):
        # patter matcher written by Cosmo Harrigan (bindlink.py)
        data = [os.path.expanduser
                ("~/atomspace/build/opencog/atomspace/core_types.scm"),
                os.path.expanduser
                ("~/atomspace/opencog/scm/*.scm")
                ]

        for item in data:
            load_scm(self.a, item)

        scheme_animals = \
            '''
            (InheritanceLink (ConceptNode "Frog") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Zebra") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Deer") (ConceptNode "animal"))
            (InheritanceLink (ConceptNode "Spaceship") (ConceptNode "machine"))
            '''

        scheme_eval_h(self.a, scheme_animals)
        # print self.a.get_atoms_by_type(types.Node)

        scheme_query = \
            '''
            (BindLink
                ;; The variable to be bound
                (VariableNode "$var")
                (ImplicationLink
                ;; The pattern to be searched for
                (InheritanceLink
                    (VariableNode "$var")
                    (ConceptNode "animal")
                )
                ;; The value to be returned.
                (VariableNode "$var")
                )
            )
            '''
        bind_link_handle = scheme_eval_h(self.a, scheme_query)

        # Run the above pattern and print the result
        result = bindlink(self.a, bind_link_handle)
        print "The result of pattern matching is:\n\n" + str(self.a[result])


class ExperimentCodes:
    def __init__(self, atomspace):
        self.a = atomspace
        self.test_case_maker = TestCaseMaker(self.a)
        self.toy_class = ToyClass(self.a)

    def execute(self):
        self.test_case_maker.make()
        # DEBUG: Start Toy method.
        # self.toy_class.foo()

        # DEBUG: Run RESTAPI server automatically to see my atomspace.
        rest_api_loader = RESTAPILoader(self.a)
        rest_api_loader.run()
__author__ = 'Cosmo Harrigan'

from nose.tools import *
import json
#from requests import get, put, post
#from flask.ext.testing import TestCase
#from flask import *
from opencog.atomspace import *
from web.api import RESTApi


class TestRESTApi():
    def setUp(self):
        self.uri = '/api/v1.0/'
        self.headers = {'content-type': 'application/json'}

        # Populate a test AtomSpace
        self.atomspace = AtomSpace()
        self.animal = self.atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
        self.bird = self.atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, .9))
        self.swan = self.atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, .9))
        self.swan_bird = self.atomspace.add_link(types.InheritanceLink, [self.swan, self.bird], TruthValue(1, 1))
        self.bird_animal = self.atomspace.add_link(types.InheritanceLink, [self.bird, self.animal], TruthValue(1, 1))
        self.bird.av = {'sti': 9}

        self.api = RESTApi(self.atomspace)
        self.client = self.api.test()

    def tearDown(self):
        pass
        #del self.moses

    def test_post_get_node(self):
        # Create a test node
        truthvalue = {'type': 'simple', 'details': {'strength': 0.8, 'count': 0.2}}
        atom = {'type': 'ConceptNode', 'name': 'giant_frog', 'truthvalue': truthvalue}

        post_response = self.client.post('/api/v1.0/atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == atom['type']
        assert post_result['name'] == atom['name']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(\
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(\
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)

        # Compare to the values created in the AtomSpace
        atomspace_result = self.atomspace[Handle(post_result['handle'])]
        assert Handle(post_result['handle']) == atomspace_result.h
        assert post_result['name'] == atomspace_result.name
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),\
                          float(post_result['truthvalue']['details']['count']))  == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = json.loads(get_response_handle.data)['atoms']
        assert post_result == get_result_handle

        # Get by name and compare
        name = post_result['name']
        get_response_name = self.client.get(self.uri + 'atoms?name=' + name)
        get_result_name = json.loads(get_response_name.data)['atoms']['result'][0]
        assert post_result == get_result_name

        # Get by name and type and compare
        type = post_result['type']
        get_response_name_type = self.client.get(self.uri + 'atoms?name=' + name + '&type=' + type)
        get_result_name_type = json.loads(get_response_name_type.data)['atoms']['result'][0]
        assert post_result == get_result_name_type

    def test_post_get_link(self):
        # Create a test link between swan and animal
        truthvalue = {'type': 'simple', 'details': {'strength': 0.5, 'count': 0.4}}
        atom = {'type': 'InheritanceLink', 'truthvalue': truthvalue, 'outgoing':\
            [self.swan.h.value(), self.animal.h.value()]}

        post_response = self.client.post('/api/v1.0/atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == atom['type']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(\
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(\
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)
        assert self.swan.h.value() in post_result['outgoing']
        assert self.animal.h.value() in post_result['outgoing']

        # Compare to the values created in the AtomSpace
        atomspace_result = self.atomspace[Handle(post_result['handle'])]
        assert Handle(post_result['handle']) == atomspace_result.h
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),\
                          float(post_result['truthvalue']['details']['count']))  == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = json.loads(get_response_handle.data)['atoms']
        assert post_result == get_result_handle

        # Check if the link is in the incoming set of each of the nodes
        for h in post_result['outgoing']:
            assert Handle(post_result['handle']) in [atom.h for atom in self.atomspace[Handle(h)].incoming]

    def test_put_truthvalue(self):
        pass # @todo both link and node

        """
        g = get('http://localhost:5000/api/v1.0/atoms/1')
        print json.dumps(g.json(), indent=4)
        truthvalue = {'type': 'simple', 'details': {'strength': 0.6, 'count': 0.5}}
        payload = {'truthvalue': truthvalue, 'attentionvalue': {'sti': 9, 'lti': 2}}
        p = put('http://localhost:5000/api/v1.0/atoms/1', data=json.dumps(payload), headers=headers)
        print json.dumps(p.json(), indent=4)
        g = get('http://localhost:5000/api/v1.0/atoms/1')
        print json.dumps(g.json(), indent=4)
        """

    def test_put_attentionvalue(self):
        pass

    def test_put_sti(self):
        pass

    def test_put_lti(self):
        pass

    def test_revise_link(self):
        pass

    def test_revise_node(self):
        pass

# @todo: test all the error codes
# @todo: test all the invalid value sending



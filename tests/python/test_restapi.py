__author__ = 'Cosmo Harrigan'

from nose.tools import *
import json
from opencog.atomspace import *

# Only run the unit tests if the required dependencies have been installed
# (see: https://github.com/opencog/opencog/issues/337)
try:
    from web.api.apimain import RESTAPI
except ImportError:
    import unittest
    raise unittest.SkipTest("ImportError exception: make sure the required dependencies are installed.")


class TestRESTApi():
    """
    Unit tests for the OpenCog REST API.

    See: opencog/python/web/api/apimain.py for the class definitions
    Documentation: http://wiki.opencog.org/w/REST_API
    """

    def setUp(self):
        self.uri = '/api/v1.0/'
        self.headers = {'content-type': 'application/json'}

        # Populate a test AtomSpace
        self.atomspace = AtomSpace()
        self.animal = self.atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
        self.bird = self.atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, .9))
        self.swan = self.atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, .9))
        self.swan_bird = self.atomspace.add_link(types.InheritanceLink, [self.swan, self.bird], TruthValue(1, .9))
        self.bird_animal = self.atomspace.add_link(types.InheritanceLink, [self.bird, self.animal], TruthValue(1, .9))
        self.bird.av = {'sti': 9}

        self.api = RESTAPI(self.atomspace)
        self.client = self.api.test()

    def tearDown(self):
        del self.api
        del self.client

    def test_post_and_get_node(self):
        # Create a test node
        truthvalue = {'type': 'simple', 'details': {'strength': 0.08, 'count': 0.2}}
        atom = {'type': 'ConceptNode', 'name': 'giant_frog', 'truthvalue': truthvalue}

        post_response = self.client.post(self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == atom['type']
        assert post_result['name'] == atom['name']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)

        # Compare to the values created in the AtomSpace
        atomspace_result = self.atomspace[Handle(post_result['handle'])]
        assert Handle(post_result['handle']) == atomspace_result.h
        assert post_result['name'] == atomspace_result.name
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),
                          float(post_result['truthvalue']['details']['count'])) == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = json.loads(get_response_handle.data)['atoms']
        assert post_result == get_result_handle

        # Get by name and compare
        name = post_result['name']
        get_response_name = self.client.get(self.uri + 'atoms?name=' + name)
        get_result_name = json.loads(get_response_name.data)['result']['atoms'][0]
        assert post_result == get_result_name

        # Get by name and type and compare
        type = post_result['type']
        get_response_name_type = self.client.get(self.uri + 'atoms?name=' + name + '&type=' + type)
        get_result_name_type = json.loads(get_response_name_type.data)['result']['atoms'][0]
        assert post_result == get_result_name_type

    def test_post_and_get_link(self):
        # Create a test link between swan and animal
        truthvalue = {'type': 'simple', 'details': {'strength': 0.5, 'count': 0.4}}
        atom = {'type': 'InheritanceLink', 'truthvalue': truthvalue, 'outgoing':
            [self.swan.h.value(), self.animal.h.value()]}

        post_response = self.client.post(self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == atom['type']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)
        assert self.swan.h.value() in post_result['outgoing']
        assert self.animal.h.value() in post_result['outgoing']

        # Compare to the values created in the AtomSpace
        atomspace_result = self.atomspace[Handle(post_result['handle'])]
        assert Handle(post_result['handle']) == atomspace_result.h
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),
                          float(post_result['truthvalue']['details']['count'])) == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = json.loads(get_response_handle.data)['atoms']
        assert post_result == get_result_handle

        # Check if the link is in the incoming set of each of the nodes
        for h in post_result['outgoing']:
            assert Handle(post_result['handle']) in [atom.h for atom in self.atomspace[Handle(h)].incoming]

    def test_put_and_get_tv_av_node(self):
        atom = self.swan
        truthvalue = {'type': 'simple', 'details': {'strength': 0.005, 'count': 0.8}}
        attentionvalue = {'sti': 9, 'lti': 2, 'vlti': True}
        atom_update = {'truthvalue': truthvalue, 'attentionvalue': attentionvalue}
        put_response = self.client.put(self.uri + 'atoms/' + str(atom.h.value()), data=json.dumps(atom_update), headers=self.headers)
        put_result = json.loads(put_response.data)['atoms']

        # Verify values returned by the PUT request
        assert put_result['handle'] == atom.h.value()
        assert_almost_equals(
            float(put_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(put_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)
        assert put_result['attentionvalue']['sti'] == attentionvalue['sti']
        assert put_result['attentionvalue']['lti'] == attentionvalue['lti']
        assert put_result['attentionvalue']['vlti'] == attentionvalue['vlti']

        # Compare to the values updated in the AtomSpace
        atomspace_result = self.atomspace[Handle(put_result['handle'])]
        assert Handle(put_result['handle']) == atomspace_result.h
        assert types.__dict__.get(put_result['type']) == atomspace_result.type
        assert TruthValue(float(put_result['truthvalue']['details']['strength']),
                          float(put_result['truthvalue']['details']['count'])) == atomspace_result.tv
        assert put_result['attentionvalue'] == atomspace_result.av

        # Get by handle and compare
        get_response = self.client.get(self.uri + 'atoms/' + str(atom.h.value()))
        get_result = json.loads(get_response.data)['atoms']
        assert put_result == get_result

    def test_put_and_get_tv_av_link(self):
        atom = self.bird_animal
        truthvalue = {'type': 'simple', 'details': {'strength': 0.9, 'count': 0.95}}
        attentionvalue = {'sti': 6, 'lti': 3, 'vlti': True}
        atom_update = {'truthvalue': truthvalue, 'attentionvalue': attentionvalue}
        put_response =\
            self.client.put(self.uri + 'atoms/' + str(atom.h.value()), data=json.dumps(atom_update), headers=self.headers)
        put_result = json.loads(put_response.data)['atoms']

        # Verify values returned by the PUT request
        assert put_result['handle'] == atom.h.value()
        assert_almost_equals(
            float(put_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(put_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)
        assert put_result['attentionvalue']['sti'] == attentionvalue['sti']
        assert put_result['attentionvalue']['lti'] == attentionvalue['lti']
        assert put_result['attentionvalue']['vlti'] == attentionvalue['vlti']

        # Compare to the values updated in the AtomSpace
        atomspace_result = self.atomspace[Handle(put_result['handle'])]
        assert Handle(put_result['handle']) == atomspace_result.h
        assert types.__dict__.get(put_result['type']) == atomspace_result.type
        assert TruthValue(float(put_result['truthvalue']['details']['strength']),
                          float(put_result['truthvalue']['details']['count'])) == atomspace_result.tv
        assert put_result['attentionvalue'] == atomspace_result.av

        # Get by handle and compare
        get_response = self.client.get(self.uri + 'atoms/' + str(atom.h.value()))
        get_result = json.loads(get_response.data)['atoms']
        assert put_result == get_result

    def test_post_revise_existing_node(self):
        # Attempt to create a node, where a node already exists with that name and type. Should revise existing node.
        existing_atom = self.bird

        truthvalue = {'type': 'simple', 'details': {'strength': 0.1, 'count': 0.95}}
        atom = {'type': 'ConceptNode', 'name': 'bird', 'truthvalue': truthvalue}

        post_response = self.client.post(self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)

        # Compare to the values updated in the AtomSpace
        assert post_result['handle'] == existing_atom.h.value()
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),
                          float(post_result['truthvalue']['details']['count'])) == existing_atom.tv

    def test_post_revise_existing_link(self):
        # Attempt to create a link, where a link already exists with that name and outgoing set.
        # Should revise existing link.
        existing_atom = self.bird_animal
        truthvalue = {'type': 'simple', 'details': {'strength': 0.1, 'count': 0.95}}
        outgoing = [a.h.value() for a in existing_atom.out]
        atom = {'type': 'InheritanceLink', 'truthvalue': truthvalue, 'outgoing': outgoing}

        post_response = self.client.post(self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['outgoing'] == outgoing
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']), truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']), truthvalue['details']['count'], places=5)

        # Compare to the values updated in the AtomSpace
        assert post_result['handle'] == existing_atom.h.value()
        assert TruthValue(float(post_result['truthvalue']['details']['strength']),
                          float(post_result['truthvalue']['details']['count'])) == existing_atom.tv

    @raises(IndexError)
    def test_delete_node(self):
        atom = self.swan
        handle = atom.h.value()
        get_response = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result = json.loads(get_response.data)['atoms']

        delete_response = self.client.delete(self.uri + 'atoms/' + str(atom.h.value()))
        delete_result = json.loads(delete_response.data)['result']

        assert delete_result['success']
        assert delete_result['handle'] == get_result['handle']

        # Confirm the atom isn't contained in the AtomSpace anymore
        assert_raises(self.atomspace[atom.h], IndexError)

    @raises(IndexError)
    def test_delete_link(self):
        atom = self.bird_animal
        handle = atom.h.value()
        get_response = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result = json.loads(get_response.data)['atoms']

        delete_response = self.client.delete(self.uri + 'atoms/' + str(atom.h.value()))
        delete_result = json.loads(delete_response.data)['result']

        assert delete_result['success']
        assert delete_result['handle'] == get_result['handle']

        # Confirm the atom isn't contained in the AtomSpace anymore
        assert_raises(self.atomspace[atom.h], IndexError)


__author__ = 'Cosmo Harrigan'

from nose.tools import *
import json
from opencog.atomspace import *

# Only run the unit tests if the required dependencies have been installed
# (see: https://github.com/opencog/opencog/issues/337)
try:
    from web.api.apimain import RESTAPI
    from web.api.utilities import count_to_confidence  # Temporary hack
except ImportError:
    import unittest
    raise unittest.SkipTest("ImportError exception: make sure the required "
                            "dependencies are installed.")


class TestRESTApi():
    """
    Unit tests for the OpenCog REST API.

    See: opencog/python/web/api/apimain.py for the class definitions
    Documentation: http://wiki.opencog.org/w/REST_API
    """

    def setUp(self):
        self.uri = '/api/v1.1/'
        self.headers = {'content-type': 'application/json'}

        # Populate a test AtomSpace
        self.atomspace = AtomSpace()
        self.animal = self.atomspace.add_node(
            types.ConceptNode, 'animal', TruthValue(.1, 0.0011860914528369904))
        self.bird = self.atomspace.add_node(
            types.ConceptNode, 'bird', TruthValue(.01, 0.0011237357975915074))
        self.swan = self.atomspace.add_node(
            types.ConceptNode, 'swan', TruthValue(.001, 0.0011237357975915074))
        self.frog = self.atomspace.add_node(
            types.ConceptNode, 'frog', TruthValue(.001, 0.7142857313156128))
        self.swan_bird = self.atomspace.add_link(
            types.InheritanceLink, [self.swan, self.bird], TruthValue(1, 0.0011237357975915074))
        self.bird_animal = self.atomspace.add_link(
            types.InheritanceLink, [self.bird, self.animal], TruthValue(1, 0.0011237357975915074))
        self.bird.sti = 9
        self.swan.sti = 9

        self.api = RESTAPI(self.atomspace)
        self.client = self.api.test()

    def tearDown(self):
        del self.api
        del self.client


    def mkatom(self, json_atom):
        post_response = self.client.post(self.uri + 'atoms',
                                         data=json.dumps(json_atom),
                                         headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']
        return post_result

    def mkswan(self):
        return self.mkatom(
                {'type': 'ConceptNode', 'name': 'swan',
                'truthvalue': {'type': 'simple',
                'details': {'strength': 0.001, 'count': 0.9}}})

    def mkbird(self):
        return self.mkatom(
                {'type': 'ConceptNode', 'name': 'bird',
                'truthvalue': {'type': 'simple',
                'details': {'strength': 0.01, 'count': 0.9}}})

    def mkanimal(self):
        return self.mkatom(
                {'type': 'ConceptNode', 'name': 'animal',
                'truthvalue': {'type': 'simple',
                'details': {'strength': 0.1, 'count': 0.95}}})

    def mkbird_animal(self):
        jbird = self.mkbird()
        janimal = self.mkanimal()
        return self.mkatom(
            {'type': 'InheritanceLink', 'truthvalue':
            {'type': 'simple', 'details': {'strength': 1.0, 'count': 0.9}},
             'outgoing': [jbird['handle'], janimal['handle']]})

    def get_atom(self, handle):
        get_response_handle = \
            self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = \
            json.loads(get_response_handle.data)['result']['atoms'][0]
        return get_result_handle

    def test_a_post_and_get_node(self):
        # Create a test node
        truthvalue = {'type': 'simple',
                      'details': {'strength': 0.08, 'count': 0.2}}
        jatom = {'type': 'ConceptNode',
                'name': 'giant_frog',
                'truthvalue': truthvalue}

        post_response = self.client.post(self.uri + 'atoms',
                                         data=json.dumps(jatom),
                                         headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == jatom['type']
        assert post_result['name'] == jatom['name']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)

        # Compare to the values created in the AtomSpace
        frog = self.atomspace.add_node(
            types.ConceptNode, 'giant_frog',
            TruthValue(.08, count_to_confidence(0.2)))
        atomspace_result = frog
        assert post_result['name'] == atomspace_result.name
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(
            float(post_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(post_result['truthvalue']['details']['count']))) \
            == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = \
            self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = \
            json.loads(get_response_handle.data)['result']['atoms'][0]
        assert post_result == get_result_handle

        # Get by name and compare
        name = post_result['name']
        get_response_name = self.client.get(self.uri + 'atoms?name=' + name)
        get_result_name = \
            json.loads(get_response_name.data)['result']['atoms'][0]
        assert post_result == get_result_name

        # Get by name and type and compare
        type = post_result['type']
        get_response_name_type = \
            self.client.get(self.uri + 'atoms?name=' + name + '&type=' + type)
        get_result_name_type = \
            json.loads(get_response_name_type.data)['result']['atoms'][0]
        assert post_result == get_result_name_type

    def test_b_post_and_get_link(self):
        jswan = self.mkswan()
        janimal = self.mkanimal()

        # Create a test link between swan and animal
        truthvalue = \
            {'type': 'simple', 'details': {'strength': 0.5, 'count': 0.4}}
        atom = {'type': 'InheritanceLink', 'truthvalue': truthvalue,
                'outgoing': [jswan['handle'], janimal['handle']]}

        post_response = self.client.post(
            self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['type'] == atom['type']
        assert post_result['truthvalue']['type'] == truthvalue['type']
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)
        assert jswan['handle'] in post_result['outgoing']
        assert janimal['handle'] in post_result['outgoing']

        # Compare to the values created in the AtomSpace
        swan_animal = self.atomspace.add_link(
            types.InheritanceLink, [self.swan, self.animal],
            TruthValue(0.5, count_to_confidence(0.4)))

        atomspace_result = swan_animal
        assert types.__dict__.get(post_result['type']) == atomspace_result.type
        assert TruthValue(
            float(post_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(post_result['truthvalue']['details']['count']))) \
            == atomspace_result.tv

        # Get by handle and compare
        handle = post_result['handle']
        get_response_handle = \
            self.client.get(self.uri + 'atoms/' + str(handle))
        get_result_handle = \
            json.loads(get_response_handle.data)['result']['atoms'][0]
        assert post_result == get_result_handle

        # Check if the link is in the incoming set of each of the nodes
        jswan = self.get_atom(jswan['handle'])
        janimal = self.get_atom(janimal['handle'])
        for h in post_result['outgoing']:
            assert post_result['handle'] in jswan['incoming']
            assert post_result['handle'] in janimal['incoming']

    def test_c_put_and_get_tv_av_node(self):
        jswan = self.mkswan()

        truthvalue = \
            {'type': 'simple', 'details': {'strength': 0.005, 'count': 0.8}}
        attentionvalue = {'sti': 9, 'lti': 2, 'vlti': True}
        atom_update = \
            {'truthvalue': truthvalue, 'attentionvalue': attentionvalue}
        put_response = self.client.put(
            self.uri + 'atoms/' + str(jswan['handle']),
            data=json.dumps(atom_update),
            headers=self.headers)
        put_result = json.loads(put_response.data)['atoms']

        # Verify values returned by the PUT request
        assert put_result['handle'] == jswan['handle']
        assert_almost_equals(
            float(put_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(put_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)
        assert put_result['attentionvalue']['sti'] == attentionvalue['sti']
        assert put_result['attentionvalue']['lti'] == attentionvalue['lti']
        assert put_result['attentionvalue']['vlti'] == attentionvalue['vlti']

        # Compare to the values updated in the AtomSpace
        atomspace_result = self.swan
        assert types.__dict__.get(put_result['type']) == atomspace_result.type
        assert TruthValue(
            float(put_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(put_result['truthvalue']['details']['count']))) \
            == atomspace_result.tv
        assert put_result['attentionvalue'] == atomspace_result.av

        # Get by handle and compare
        get_response = \
            self.client.get(self.uri + 'atoms/' + str(jswan['handle']))
        get_result = json.loads(get_response.data)['result']['atoms'][0]
        assert put_result == get_result

    def test_d_put_and_get_tv_av_link(self):
        jatom = self.mkbird_animal()
        truthvalue = \
            {'type': 'simple', 'details': {'strength': 0.9, 'count': 0.95}}
        attentionvalue = {'sti': 6, 'lti': 3, 'vlti': True}
        atom_update = \
            {'truthvalue': truthvalue, 'attentionvalue': attentionvalue}
        put_response =\
            self.client.put(
                self.uri + 'atoms/' + str(jatom['handle']),
                data=json.dumps(atom_update),
                headers=self.headers)
        put_result = json.loads(put_response.data)['atoms']

        # Verify values returned by the PUT request
        assert put_result['handle'] == jatom['handle']
        assert_almost_equals(
            float(put_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(put_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)
        assert put_result['attentionvalue']['sti'] == attentionvalue['sti']
        assert put_result['attentionvalue']['lti'] == attentionvalue['lti']
        assert put_result['attentionvalue']['vlti'] == attentionvalue['vlti']

        # Compare to the values updated in the AtomSpace
        atomspace_result = self.bird_animal
        assert types.__dict__.get(put_result['type']) == atomspace_result.type
        assert TruthValue(
            float(put_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(put_result['truthvalue']['details']['count']))) \
            == atomspace_result.tv
        assert put_result['attentionvalue'] == atomspace_result.av

        # Get by handle and compare
        get_response = \
            self.client.get(self.uri + 'atoms/' + str(jatom['handle']))
        get_result = json.loads(get_response.data)['result']['atoms'][0]
        assert put_result == get_result

    def test_e_post_revise_existing_node(self):
        # Attempt to create a node, where a node already exists with that name
        # and type. Should revise existing node.
        existing_atom = self.bird

        truthvalue = \
            {'type': 'simple', 'details': {'strength': 0.1, 'count': 0.95}}
        atom = \
            {'type': 'ConceptNode', 'name': 'bird', 'truthvalue': truthvalue}

        post_response = self.client.post(self.uri + 'atoms',
                                         data=json.dumps(atom),
                                         headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)

        # Compare to the values updated in the AtomSpace
        assert TruthValue(
            float(post_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(post_result['truthvalue']['details']['count']))) \
               == existing_atom.tv

    def test_f_post_revise_existing_link(self):
        # Attempt to create a link, where a link already exists with that name
        # and outgoing set.
        # Should revise existing link.
        existing_atom = self.bird_animal
        jbird_animal = self.mkbird_animal()
        truthvalue = \
            {'type': 'simple', 'details': {'strength': 0.1, 'count': 0.95}}
        outgoing = jbird_animal['outgoing']
        atom = {'type': 'InheritanceLink',
                'truthvalue': truthvalue,
                'outgoing': outgoing}

        post_response = self.client.post(
            self.uri + 'atoms', data=json.dumps(atom), headers=self.headers)
        post_result = json.loads(post_response.data)['atoms']

        # Verify values returned by the POST request
        assert post_result['outgoing'] == outgoing
        assert_almost_equals(
            float(post_result['truthvalue']['details']['strength']),
            truthvalue['details']['strength'], places=5)
        assert_almost_equals(
            float(post_result['truthvalue']['details']['count']),
            truthvalue['details']['count'], places=5)

        # Compare to the values updated in the AtomSpace
        assert TruthValue(
            float(post_result['truthvalue']['details']['strength']),
            count_to_confidence(
                float(post_result['truthvalue']['details']['count']))) \
               == existing_atom.tv

    # @raises(IndexError)
    def test_g_delete_node(self):
        jswan = self.mkswan()
        handle = jswan['handle']
        get_response = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result = json.loads(get_response.data)['result']['atoms'][0]

        old_size = self.atomspace.size()
        delete_response = \
            self.client.delete(self.uri + 'atoms/' + str(handle))
        delete_result = json.loads(delete_response.data)['result']

        assert delete_result['success']
        assert delete_result['handle'] == get_result['handle']

        # Confirm the atom isn't contained in the AtomSpace anymore
        # Actually, the python bindings don't provide a way to ask
        # this question. So we check the size, instead.  Should shrink
        # by 2, since both the swan, and the inhertance link get
        # deleted.
        new_size = self.atomspace.size()
        assert new_size + 2 == old_size

    # @raises(IndexError)
    def test_h_delete_link(self):
        jatom = self.mkbird_animal()
        handle = jatom['handle']
        get_response = self.client.get(self.uri + 'atoms/' + str(handle))
        get_result = json.loads(get_response.data)['result']['atoms'][0]

        old_size = self.atomspace.size()
        delete_response = \
            self.client.delete(self.uri + 'atoms/' + str(handle))
        delete_result = json.loads(delete_response.data)['result']

        assert delete_result['success']
        assert delete_result['handle'] == get_result['handle']

        # Confirm the atom isn't contained in the AtomSpace anymore
        # Actually, the python bindings don't provide a way to ask
        # this question. So we check the size, instead.  Should shrink
        # by 1, since only the inhertance link gets deleted.
        new_size = self.atomspace.size()
        assert new_size + 1 == old_size

    def test_j_get_types(self):
        # Verify that a list of valid atom types was returned
        get_response = self.client.get(self.uri + 'types')
        get_result = json.loads(get_response.data)['types']
        assert len(get_result) > 0
        assert get_result.__contains__('ConceptNode')

    def test_k_tv_filter(self):
        # Should return animal, swan_bird, bird_animal (3 atoms)
        get_response = self.client.get(self.uri + 'atoms?tvStrengthMin=0.1')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 3

        # Should return frog (1 atom)
        get_response = self.client.get(self.uri + 'atoms?tvConfidenceMin=0.7')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 1

        # Should return frog (1 atom)
        get_response = self.client.get(self.uri + 'atoms?tvCountMin=2000')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 1

    def test_l_include_incoming_outgoing(self):
        # Should return bird and swan (2 atoms)
        get_response = self.client.get(
            self.uri +
            'atoms?filterby=stirange&stimin=1&includeIncoming=false')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 2

        # Should additionally return swan_bird and bird_animal (4 atoms)
        get_response = self.client.get(
            self.uri + 'atoms?filterby=stirange&stimin=1&includeIncoming=true')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 4

        # Should additionally return animal (5 atoms)
        get_response = \
            self.client.get(
                self.uri + 'atoms?filterby=stirange&stimin=1&'
                           'includeIncoming=true&includeOutgoing=true')
        get_result = json.loads(get_response.data)['result']['atoms']
        assert len(get_result) == 5

    def test_m_scheme_command(self):
        # Test an arbitrary Scheme command to ensure the binding is working
        # properly
        # XXX Emptied because the scheme command (i.e cog-af-boundary) has been removed.
        pass 
    def test_n_dot_export(self):
        # Export the atomspace to DOT format and ensure that there is a
        # properly defined DOT header created and the correct atoms are
        # included in the description

        # TODO: The Python module "graphviz" needs to be added to ocpkg, so
        # that this dependency will be available for the continuous integration
        # system
        try:
            from graph_description import dot

            get_response = self.client.get(
                self.uri +
                'atoms?filterby=attentionalfocus&dot=True')
            get_result = json.loads(get_response.data)['result']
            assert get_result.startswith("// OpenCog Graph")
            assert "digraph" in get_result
            assert "swan" in get_result
            assert "bird" in get_result
            assert get_result.count("label") == 2
        except ImportError:
            pass

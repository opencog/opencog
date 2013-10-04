"""
REST API for OpenCog

Implemented using the Flask micro-framework and flask-restful extension
"""

__author__ = 'Cosmo Harrigan'

from flask import * # @todo remove
from flask import Flask, abort
from flask import request, make_response, url_for
from flask.views import MethodView
from flask.ext.restful import Api, Resource, reqparse, fields, marshal
from opencog.atomspace import *

# Define classes for parsing object values for serialization
class FormatHandleValue(fields.Raw):
    def format(self, value):
        return value.value()

class FormatHandleList(fields.Raw):
    def format(self, value):
        handles = []
        for elem in value:
            handles.append(elem.h.value())
        return handles

class FormatAtomList(fields.Raw):
    def format(self, value):

        {
            'complete': True,
            'skipped':
"skipped":0,
"total":10,
        }
        'complete', 'skipped', 'total', 'result'

tv_fields = {
    'strength': fields.Float(attribute='mean'),
    'confidence': fields.Float(attribute='confidence'),
    'count': fields.Float(attribute='count')
}

av_fields = {
    'sti': fields.Integer(attribute='sti'),
    'lti': fields.Integer(attribute='lti'),
    'vlti': fields.Boolean(attribute='vlti')
}

# Define mapping of object attributes to API results
atom_fields = {
    'handle': FormatHandleValue(attribute='h'),
    'type': fields.String(attribute='type_name'),
    'name': fields.String,
    'outgoing': FormatHandleList(attribute='outgoing'),
    'incoming': FormatHandleList(attribute='incoming'),
    'truthvalue': fields.Nested(tv_fields, attribute='tv'),
    'attentionvalue': fields.Nested(av_fields, attribute='av')
}
'''
>>> resource_fields = {'name': fields.String}
>>> resource_fields['address'] = {}
>>> resource_fields['address']['line 1'] = fields.String(attribute='addr1')
atom_list_fields = {
    'complete': fields.String
}
'''

class AtomListAPI(Resource):
    @classmethod
    def set_atomspace(cls, atomspace):
        cls.atomspace = atomspace

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('type', type=str, location='args', choices=types.__dict__.keys())
        super(AtomListAPI, self).__init__()

    def get(self):
        args = self.reqparse.parse_args()
        type_lookup = args.get('type')
        if type_lookup is None:
            atoms = self.atomspace.get_atoms_by_type(types.Atom)
        else:
            atoms = self.atomspace.get_atoms_by_type(types.__dict__.get(type_lookup))

        # @todo: Implement pagination
        # 'complete', 'skipped', 'total', 'result'
        return {'atoms': map(lambda t: marshal(t, atom_fields), atoms)}


class AtomAPI(Resource):
    @classmethod
    def set_atomspace(cls, atomspace):
        cls.atomspace = atomspace

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('type', type=str, choices=types.__dict__.keys())
        self.reqparse.add_argument('name', type=str)
        super(AtomAPI, self).__init__()

    def get(self, id):
        try:
            atom = self.atomspace[Handle(id)]
        except IndexError:
            abort(404)

        return {'atom': marshal(atom, atom_fields)}

    '''
    @todo PUT
    "update the STI, LTI, or TV of an atom with JSON message",
    "Usage: json-update-atom handle {JSON}\n\n"
    "   Update an atom based on a JSON message with format:\n"
    "   { \"sti\": STI } \n"
    "   { \"lti\": LTI } \n"
    "   { \"tv\": {tv details} } \n",
    '''

    # @todo: support PATCH for partial update
    #def put(self, id):

    '''
    @todo POST
    "Usage: json-create-atom<CRLF>JSON<CRLF><Ctrl-D><CRLF>\n\n"
            "   Create an atom based on the JSON format:\n"
            "   { \"type\": TYPENAME, \"name\": NAME, \n"
            "     \"outgoing\": [ UUID1, UUID2 ... ], \n"
            "     \"truthvalue\": {\"simple|composite|count|indefinite\":\n"
            "          [truthvalue details] } \n",
            true, false

            TruthValue: See opencog/web/JsonUtil.cc JSONToTV method
    '''

    # @todo what happens if i post to an existing atom

    def post(self):
        data = reqparse.request.get_json()
        type_data = types.__dict__.get(data['type'])
        truthvalue_data = TruthValue(data['truthvalue']['strength'], data['truthvalue']['count'])

        if type_data is None or truthvalue_data is None or data['name'] is None:
            # @todo: check what is the proper error type
            abort(500)

        try:
            atom = self.atomspace.add(t=type_data, name=data['name'], tv=truthvalue_data)
        except TypeError:
            # @todo: check what is the proper error type
            print "Debug: TypeError - check your parameters"
            abort(500)

        print 'Atom created:'
        print atom
        return {'atom': marshal(atom, atom_fields)}

    def delete(self, id):
        try:
            atom = self.atomspace[Handle(id)]
        except IndexError:
            abort(404)

        success = self.atomspace.remove(atom)
        return {'result': success}


class RESTApi(object):
    def __init__(self, atomspace):
        ######### For testing purposes, populate an AtomSpace with nodes & links:
        self.atomspace = atomspace

        # Initialize the web server and set the routing
        self.app = Flask(__name__, static_url_path="")
        self.api = Api(self.app)
        AtomListAPI.set_atomspace(self.atomspace)
        AtomAPI.set_atomspace(self.atomspace)
        self.api.add_resource(AtomListAPI, '/api/v1.0/atoms', endpoint='atoms')
        self.api.add_resource(AtomAPI, '/api/v1.0/atoms/<int:id>', '/api/v1.0/atoms', endpoint='atom')

    def run(self):
        self.app.run(debug=True)

if __name__ == '__main__':
    test_atomspace = AtomSpace()

    animal = test_atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
    bird = test_atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, .9))
    swan = test_atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, .9))
    swan_bird = test_atomspace.add_link(types.InheritanceLink, [swan, bird], TruthValue(1, 1))
    bird_animal = test_atomspace.add_link(types.InheritanceLink, [bird, animal], TruthValue(1, 1))
    bird.av = {'sti': 9}

    api = RESTApi(test_atomspace)
    api.run()

'''
now
# @todo: Return JSON errors\

Specify the type of the TruthValue returned in the JSON
Specify the type of the TruthValue passed in the JSON
        # @todo: ? Linas says handle isn't supposed to be the only id - how about a get method using name/type pair?
        # @todo: support setting outgoing links (and incoming?)
        # @todo: Verify that the 'strength' and 'count' terms are proper for TruthValue
        # @todo: Document how to test the API using curl and Python 'request'

Wrap the results as if there were pagination, even though there isn't with 'complete', 'skipped', 'total', 'result' attributes
'''

# Later:
# @todo: support multiple TruthValue types in the get and post and update
# @todo: create an enumeration of valid TruthValue types
# @todo: get methods for TruthValue: see opencog\web\GetAtomRequest.cc tvToJSON method

'''
Types:
SIMPLE_TRUTH_VALUE
COUNT_TRUTH_VALUE
INDEFINITE_TRUTH_VALUE
COMPOSITE_TRUTH_VALUE

As noted in the cdef for TruthValue in opencog\cython\opencog\atomspace_details.pyx, only SimpleTruthValue has been implemented so far in the Python bindings
'''

"""
Testing:

import json
from opencog.atomspace import *
from requests import *

# GET
#  LIST
g = get('http://localhost:5000/api/v1.0/atoms')
print json.dumps(g.json(), indent=2)

#  ONE ATOM
g = get('http://localhost:5000/api/v1.0/atoms/1')
print json.dumps(g.json(), indent=2)

# POST
payload = {'type': 'ConceptNode', 'name': 'ugly_frog'}
headers = {'content-type': 'application/json'}
r = post('http://localhost:5000/api/v1.0/atoms/', data=json.dumps(payload), headers=headers)
print json.dumps(r.json(), indent=2)

"""
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


class FormatTruthValue(fields.Raw):
    def format(self, value):
        return {
            'type': 'simple',
            'details': marshal(value, tv_fields)
        }


class FormatAtomList(object):
    @staticmethod
    def format(atoms):
        return {
            # @todo: Add pagination (http://flask.pocoo.org/snippets/44/)
            'complete': True,
            'skipped': 0,
            'total': len(atoms),
            'result': map(lambda t: marshal(t, atom_fields), atoms)
        }

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
    'truthvalue': FormatTruthValue(attribute='tv'),
    'attentionvalue': fields.Nested(av_fields, attribute='av')
}


class AtomListAPI(Resource):
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

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

        return {'atoms': FormatAtomList.format(atoms)}

    # @todo what happens if i post to an existing atom -- check!

    def post(self):
        """
        Creates a new atom. If the atom already exists, it updates the atom.
        Post a JSON object as data with the following format:

        type (required) Atom type, see http://wiki.opencog.org/w/OpenCog_Atom_types
        name (required for Node types, not allowed for Link types) Atom name
        truthvalue (required) Truth value, formatted as follows:
            type Truth value type (only 'simple' is currently available), see http://wiki.opencog.org/w/TruthValue
            details Truth value parameters, formatted as follows:
                strength
                count
        outgoing (optional) The set of arguments of the relation, formatted as a list of Atom handles
            (only valid for Links, not nodes), see http://wiki.opencog.org/w/Link#Incoming_and_Outgoing_Sets

        Examples:

        Node:
            {
                "type": "ConceptNode",
                "name": "Frog",
                "truthvalue": {
                    "type": "simple",
                    "details": {
                        "strength": 0.8,
                        "count": 0.2
                    }
                }
            }

        Link:
        @todo
            "outgoing": [
                            2,
                            3
                        ]

        Returns a JSON representation of the Atom

        Example: @todo

        """

        # Prepare the atom data and validate it
        data = reqparse.request.get_json()

        if 'type' in data:
            if type in types.__dict__:
                type = types.__dict__.get(data['type'])
            else:
                abort(400, 'Invalid request: type \'' + data['type'] + '\' is not a valid type')
        else:
            abort(400, 'Invalid request: required parameter type is missing')

        # TruthValue
        if 'truthvalue' in data:
            if 'type' in data['truthvalue']:
                tv_type = data['truthvalue']['type']
            else:
                abort(400, 'Invalid request: truthvalue object requires a type parameter')
            if 'details' in data['truthvalue']:
                if 'strength' in data['truthvalue']['details'] and 'count' in data['truthvalue']['details']:
                    tv_details = TruthValue(data['truthvalue']['details']['strength'], data['truthvalue']['details']['count'])
                else:
                    abort(400, 'Invalid request: truthvalue details object requires both a strength and count parameter')
            else:
                abort(400, 'Invalid request: truthvalue object requires a details parameter')
        else:
            abort(400, 'Invalid request: required parameter truthvalue is missing')

        # Outgoing set
        if 'outgoing' in data:
            outgoing = data['outgoing']
        else:
            outgoing = None

        # Name
        if 'name' in data:
            name = data['name']
        else:
            name = None

        # Nodes must have names
        if is_a(type, types.Node):
            if name is None:
                abort(400, 'Invalid request: node type specified and required parameter name is missing')
        # Links can't have names
        else:
            if name is not None:
                abort(400, 'Invalid request: parameter name is not allowed for link types')

        # @todo: Cython bindings don't provide support for other TruthValue types yet
        #        (see: opencog\cython\opencog\atomspace_details.pyx, opencog\cython\opencog\atomspace.pxd)
        if tv_type != 'simple':
            if tv_type in ['composite', 'count', 'indefinite']:
                # @todo: check error type
                abort(500)  # say we don't support those yet
            else:
                # @todo: check error type
                abort(500)

        try:
            atom = self.atomspace.add(t=type, name=name, tv=tv_details, out=outgoing)
        except TypeError:
            # @todo: check what is the proper error type
            print "Debug: TypeError - check your parameters"
            abort(500)

        return {'atom': marshal(atom, atom_fields)}

    def delete(self, id):
        try:
            atom = self.atomspace[Handle(id)]
        except IndexError:
            abort(404)

        succeeded = self.atomspace.remove(atom)
        return {'result': succeeded}


class AtomAPI(Resource):
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

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
    # @todo: support PATCH for partial update
    #def put(self, id):
    '''

    def put(self, id):
        """
        Update the STI, LTI or TruthValue of an atom
        """

        # Prepare the atom data
        data = reqparse.request.get_json()

        try:
            tv_type = data['truthvalue']['type']
            tv_details = TruthValue(data['truthvalue']['details']['strength'], data['truthvalue']['details']['count'])
        except KeyError:
            tv_type = None

        try:
            sti = data['attentionvalue']['sti']

            print marshal(data['attentionvalue'], av_fields)
        except KeyError:
            av = None

        # Perform validation on the atom data
        if tv_type != 'simple' and tv_type is not None:
            if tv_type in ['composite', 'count', 'indefinite']:
                # @todo: check error type
                abort(500)  # say we don't support those yet
            else:
                # @todo: check error type
                abort(500)




class RESTApi(object):
    def __init__(self, atomspace):
        ######### For testing purposes, populate an AtomSpace with nodes & links:
        self.atomspace = atomspace

        # Initialize the web server and set the routing
        self.app = Flask(__name__, static_url_path="")
        self.api = Api(self.app)
        atom_api = AtomAPI.new(self.atomspace)
        atom_list_api = AtomListAPI.new(self.atomspace)
        self.api.add_resource(atom_list_api, '/api/v1.0/atoms', endpoint='atoms')
        self.api.add_resource(atom_api, '/api/v1.0/atoms/<int:id>', endpoint='atom')

        #, '/api/v1.0/atoms'

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
# @todo: Return JSON errors

        # @todo: ? Linas says handle isn't supposed to be the only id - how about a get method using name/type pair?
        # @todo: support setting outgoing links (and incoming?)
        # @todo: Document how to test the API using curl and Python 'request'
        # @todo: check if the truth value type enumeration is complete

'''

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

@todo: UPDATE THIS WITH OUTGOING SET examples

# POST
payload = {'type': 'ConceptNode', 'name': 'ugly_frog'}
headers = {'content-type': 'application/json'}
r = post('http://localhost:5000/api/v1.0/atoms/', data=json.dumps(payload), headers=headers)
print json.dumps(r.json(), indent=2)


truthvalue = {'type': 'simple', 'details': {'strength': 0.8, 'count': 0.2}}
payload = {'type': 'ConceptNode', 'name': 'ugly_frog_prince4', 'truthvalue': truthvalue}
headers = {'content-type': 'application/json'}
r = post('http://localhost:5000/api/v1.0/atoms', data=json.dumps(payload), headers=headers)
print json.dumps(r.json(), indent=2)

"""
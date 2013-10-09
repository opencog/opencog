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


class ParseTruthValue(object):
    @staticmethod
    def parse(data):
        if 'truthvalue' in data:
            if 'type' in data['truthvalue']:
                tv_type = data['truthvalue']['type']
            else:
                abort(400, 'Invalid request: truthvalue object requires a type parameter')
        else:
            abort(400, 'Invalid request: required parameter truthvalue is missing')

        # @todo: Cython bindings implementation does not provide support for other TruthValue types yet
        #        (see: opencog\cython\opencog\atomspace_details.pyx, opencog\cython\opencog\atomspace.pxd)
        if tv_type != 'simple':
            if tv_type in ['composite', 'count', 'indefinite']:
                # @todo: check error type
                abort(400, 'Invalid request: truthvalue type \'' + tv_type + '\' is not supported')
            else:
                # @todo: check error type
                abort(400, 'Invalid request: type \'' + tv_type + '\' is not a valid truthvalue type')

        if 'details' in data['truthvalue']:
            if 'strength' in data['truthvalue']['details'] and 'count' in data['truthvalue']['details']:
                tv = TruthValue(data['truthvalue']['details']['strength'], data['truthvalue']['details']['count'])
            else:
                abort(400, 'Invalid request: truthvalue details object requires both a strength and count parameter')
        else:
            abort(400, 'Invalid request: truthvalue object requires a details parameter')

        return tv


class ParseAttentionValue(object):
    @staticmethod
    def parse(data):
        av = data['attentionvalue']
        sti = av['sti'] if 'sti' in av else None
        lti = av['lti'] if 'lti' in av else None
        vlti = av['vlti'] if 'vlti' in av else None

        return sti, lti, vlti

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
    'outgoing': FormatHandleList(attribute='out'),
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
        self.reqparse.add_argument('name', type=str, location='args')
        super(AtomListAPI, self).__init__()

    def get(self):
        args = self.reqparse.parse_args()
        type = args.get('type')
        name = args.get('name')
        if type is None and name is None:
            atoms = self.atomspace.get_atoms_by_type(types.Atom)
        elif name is None:
            atoms = self.atomspace.get_atoms_by_type(types.__dict__.get(type))
        else:
            if type is None:
                type = 'Node' # Default to Node if a name is provided without specifying a type
            atoms = self.atomspace.get_atoms_by_name(t=types.__dict__.get(type), name=name)

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
            type = data['type']
            if type in types.__dict__:
                type = types.__dict__.get(data['type'])
            else:
                abort(400, 'Invalid request: type \'' + type + '\' is not a valid type')
        else:
            abort(400, 'Invalid request: required parameter type is missing')

        # TruthValue
        tv = ParseTruthValue.parse(data)

        # Outgoing set
        if 'outgoing' in data:
            if len(data['outgoing']) > 0:
                outgoing = [Handle(h) for h in data['outgoing']]
        else:
            outgoing = None

        # Name
        name = data['name'] if 'name' in data else None

        # Nodes must have names
        if is_a(type, types.Node):
            if name is None:
                abort(400, 'Invalid request: node type specified and required parameter name is missing')
        # Links can't have names
        else:
            if name is not None:
                abort(400, 'Invalid request: parameter name is not allowed for link types')

        try:
            atom = self.atomspace.add(t=type, name=name, tv=tv, out=outgoing)

        except TypeError:
            abort(500, 'Error while processing your request. Check your parameters.')

        return {'atoms': marshal(atom, atom_fields)}

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

        return {'atoms': marshal(atom, atom_fields)}

    def put(self, id):
        """
        Update the STI, LTI or TruthValue of an atom
        """

        # @todo: check if handle exists

        # Prepare the atom data
        data = reqparse.request.get_json()

        if 'truthvalue' not in data and 'attentionvalue' not in data:
            abort(400, 'Invalid request: you must include a truthvalue or attentionvalue parameter')

        if 'truthvalue' in data:
            tv = ParseTruthValue.parse(data)
            self.atomspace.set_tv(h=Handle(id), val=tv)

        if 'attentionvalue' in data:
            (sti, lti, vlti) = ParseAttentionValue.parse(data)
            self.atomspace.set_av(h=Handle(id), sti=sti, lti=lti, vlti=vlti)


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

    def test(self):
        return self.app.test_client()

# @todo: remove this test code
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
@todo separate the parsing and main api into separate files
@todo use named tuples in the parsing

@todo see if the atomspace throws when giving weird values as tv or av updates
@todo handle put of invalid handle
@todo check get invalid type, get invalid name

@todo write unit tests
@todo add dependencies to ocpkg, detect dependency existence in cmake
@todo add get by name, get by type
@todo search by incoming or outgoing set
@todo finish fixing the errors

@todo: Document how to access the API using curl and Python 'request'
@todo: check if the truth value type enumeration is complete

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


# PUT
g = get('http://localhost:5000/api/v1.0/atoms/1')
print json.dumps(g.json(), indent=4)
truthvalue = {'type': 'simple', 'details': {'strength': 0.6, 'count': 0.5}}
payload = {'truthvalue': truthvalue, 'attentionvalue': {'sti': 9, 'lti': 2}}
p = put('http://localhost:5000/api/v1.0/atoms/1', data=json.dumps(payload), headers=headers)
print json.dumps(p.json(), indent=4)
g = get('http://localhost:5000/api/v1.0/atoms/1')
print json.dumps(g.json(), indent=4)


"""
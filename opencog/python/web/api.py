"""
REST API for OpenCog
Implemented using the Flask micro-framework and flask-restful extension
"""

__author__ = 'Cosmo Harrigan'

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

tv_fields = {
    # @todo: verify the mappings of the following terminology
    'strength': fields.Float(attribute='mean'),
    'confidence': fields.Float(attribute='confidence'),
    'count': fields.Float(attribute='count')
}

av_fields = {
    'sti': fields.Float(attribute='sti'),
    'lti': fields.Float(attribute='lti'),
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


class AtomListAPI(Resource):
    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('type', type=str, location='args')
        super(AtomListAPI, self).__init__()

    def get(self):
        args = self.reqparse.parse_args()
        # @todo: Find how to retrieve all atoms of all types
        # @todo: Find how to retrieve atoms of a specific type using the 'type' arg
        atoms = atomspace.get_atoms_by_type(types.ConceptNode)

        # @todo: Implement pagination with 'complete', 'skipped', 'total', 'result' attributes
        return {'atoms': map(lambda t: marshal(t, atom_fields), atoms)}


class AtomAPI(Resource):
    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        # @todo: do we need to specify location='json'?
        self.reqparse.add_argument('type', type=str)
        self.reqparse.add_argument('name', type=str)
        super(AtomAPI, self).__init__()

    def get(self, id):
        try:
            atom = atomspace[Handle(id)]
        except IndexError:
            abort(404)

        '''
        # Test how to convert a type in the query string into an object of type 'atom.type'
        args = self.reqparse.parse_args()
        test_type = args.get('type', atom.type)
        print test_type
        '''

        return {'atom': marshal(atom, atom_fields)}

    # @todo: support PATCH for partial update
    #def put(self, id):
    def post(self):
        args = self.reqparse.parse_args()
        print 'Type:'
        print args.get('type')
        print 'Name:'
        print args.get('name')
        # @todo: Convert 'type' string arg to 'type' object
        # @todo: Convert 'truthvalue' arg to TruthValue
        # @todo: Convert add_node method to add method, supporting nodes & links
        # @todo: During testing, if you add a node twice, using the same name, both nodes receive the same handle. That might be a bug in the API.
        atom = atomspace.add_node(types.ConceptNode, args.get('name'), TruthValue(.5, .6))
        #atom = atomspace.add_node(args.get('type'), args.get('name'), TruthValue(.5, .6))
        print 'Atom created:'
        print atom
        return {'atom': marshal(atom, atom_fields)}

    def delete(self, id):
        try:
            atom = atomspace[Handle(id)]
        except IndexError:
            abort(404)

        success = atomspace.remove(atom)
        return { 'result': success }

######### For testing purposes, populate an AtomSpace with nodes & links:
# @todo: Move the atomspace into a parameter of the RESTApi class instanciation and allow the API classes to access it
atomspace = AtomSpace()
animal = atomspace.add_node(types.ConceptNode, 'animal', TruthValue(.1, .9))
bird = atomspace.add_node(types.ConceptNode, 'bird', TruthValue(.01, .9))
swan = atomspace.add_node(types.ConceptNode, 'swan', TruthValue(.001, .9))
swan_bird = atomspace.add_link(types.InheritanceLink, [swan, bird], TruthValue(1, 1))
bird_animal = atomspace.add_link(types.InheritanceLink, [bird, animal], TruthValue(1, 1))
#bird_animal.setav(AttentionValue(1, 1)) Why isn't this working? Maybe because the Cython test isn't passing after #305

class RESTApi(object):
    def __init__(self):
        self.app = Flask(__name__, static_url_path="")
        self.api = Api(self.app)
        self.api.add_resource(AtomListAPI, '/api/v1.0/atoms', '/api/v1.0/atoms/', endpoint = 'atoms')
        self.api.add_resource(AtomAPI, '/api/v1.0/atoms/<int:id>', '/api/v1.0/atoms/', endpoint='atom')
    def run(self):
        self.app.run(debug = True)

if __name__ == '__main__':
    api = RESTApi()
    api.run()


__author__ = 'Cosmo Harrigan'

from flask import abort, json, current_app
from flask.ext.restful import Resource, reqparse, marshal
from opencog.atomspace import Handle
from mappers import *


class AtomCollectionAPI(Resource):
    # This is because of https://github.com/twilio/flask-restful/issues/134
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('type', type=str, location='args', choices=types.__dict__.keys())
        self.reqparse.add_argument('name', type=str, location='args')
        self.reqparse.add_argument('callback', type=str, location='args')
        super(AtomCollectionAPI, self).__init__()

    def get(self):
        """
        Returns a list of atoms matching the specified criteria
        Uri: atoms?type=[type]&name=[name]&callback=[callback]

        :param type: (optional) Atom type, see http://wiki.opencog.org/w/OpenCog_Atom_types
        :param name: (optional, not allowed for Link types) Atom name
        If neither type or name are provided, all atoms will be retrieved
        :param callback: (optional) JavaScript callback function for JSONP support

        :return result: Returns a JSON representation of an atom list.
        Example:

        {
          'result':
          {
            'complete': 'true',
            'skipped': 'false',
            'total': 10,
            'atoms':
              [
                { 'handle': 6,
                  'name': '',
                  'type': 'InheritanceLink',
                  'outgoing': [2, 1],
                  'incoming': [],
                  'truthvalue':
                    {
                      'type': 'simple',
                      'details':
                        {
                          'count': '0.4000000059604645',
                          'confidence': '0.0004997501382604241',
                          'strength': '0.5'
                        }
                    }
                  'attentionvalue':
                    {
                      'lti': 0,
                      'sti': 0,
                      'vlti': false
                    }
                },
                     ...
              ]
          }
        }
        """

        args = self.reqparse.parse_args()
        type = args.get('type')
        name = args.get('name')
        callback = args.get('callback')
                  
        if type is None and name is None:
            atoms = self.atomspace.get_atoms_by_type(types.Atom)
        elif name is None:
            atoms = self.atomspace.get_atoms_by_type(types.__dict__.get(type))
        else:
            if type is None:
                type = 'Node'
            atoms = self.atomspace.get_atoms_by_name(t=types.__dict__.get(type), name=name)

        atom_list = AtomListResponse(atoms)
        json_data = {'result': atom_list.format()}
        
        # if callback function supplied, pad the JSON data (i.e. JSONP):
        if callback is not None:
            response = str(callback) + '(' + json.dumps(json_data) + ');'
            return current_app.response_class(response, mimetype = 'application/javascript')
        else:
            return current_app.response_class(json.dumps(json_data), mimetype = 'application/json')
        
    

    def post(self):
        """
        Creates a new atom. If the atom already exists, it updates the atom.
        Uri: atoms

        Include data with the POST request providing a JSON representation of the atom.
        Valid data elements:

        type (required) Atom type, see http://wiki.opencog.org/w/OpenCog_Atom_types
        name (required for Node types, not allowed for Link types) Atom name
        truthvalue (required) TruthValue, formatted as follows:
            type (required) TruthValue type (only 'simple' is currently available), see http://wiki.opencog.org/w/TruthValue
            details (required) TruthValue parameters, formatted as follows:
                strength (required)
                count (required)
        outgoing (optional) The set of arguments of the relation, formatted as a list of Atom handles
            (only valid for Links, not nodes), see http://wiki.opencog.org/w/Link#Incoming_and_Outgoing_Sets

        Examples:

        Node: {
                'type': 'ConceptNode',
                'name': 'Frog',
                'truthvalue':
                  {
                    'type': 'simple',
                    'details':
                      {
                        'strength': 0.8,
                        'count': 0.2
                      }
                  }
              }

        Link: {
                'type': 'InheritanceLink',
                'outgoing': [1, 2],
                'truthvalue':
                  {
                    'type': 'simple',
                    'details':
                      {
                        'strength': 0.5,
                        'count': 0.4
                      }
                  }
              }

        :return atoms: Returns a JSON representation of an atom list containing the atom. Example:
        {
          'atoms':
          {
            'handle': 6,
            'name': '',
            'type': 'InheritanceLink',
            'outgoing': [2, 1],
            'incoming': [],
            'truthvalue':
              {
                'type': 'simple',
                'details':
                  {
                    'count': '0.4000000059604645',
                    'confidence': '0.0004997501382604241',
                    'strength': '0.5'
                  }
              },
            'attentionvalue':
              {
                'lti': 0,
                'sti': 0,
                'vlti': false
              }
          }
        }
        """

        # Prepare the atom data and validate it
        data = reqparse.request.get_json()

        if 'type' in data:
            if data['type'] in types.__dict__:
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

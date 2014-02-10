__author__ = 'Cosmo Harrigan'

from flask import abort, json, current_app
from flask.ext.restful import Resource, reqparse, marshal
from opencog.atomspace import Handle
from mappers import *
from flask.ext.restful.utils import cors

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
        self.reqparse.add_argument('filterby', type=str, location='args',
                                   choices=['stirange', 'attentionalfocus'])
        self.reqparse.add_argument('stimin', type=int, location='args')
        self.reqparse.add_argument('stimax', type=int, location='args')
        self.reqparse.add_argument('tvStrengthMin', type=float, location='args')
        self.reqparse.add_argument('tvConfidenceMin', type=float, location='args')
        self.reqparse.add_argument('tvCountMin', type=float, location='args')
        self.reqparse.add_argument('includeIncoming', type=str, location='args',
                                   choices=['true', 'false', 'True', 'False', '0', '1'])
        self.reqparse.add_argument('includeOutgoing', type=str, location='args',
                                   choices=['true', 'false', 'True', 'False', '0', '1'])

        super(AtomCollectionAPI, self).__init__()

    # Set CORS headers to allow cross-origin access (https://github.com/twilio/flask-restful/pull/131):
    @cors.crossdomain(origin='*')
    def get(self):
        """
        Returns a list of atoms matching the specified criteria
        Uri: atoms?type=[type]&name=[name]&filterby=[filterby]&tvStrengthMin=[tvStrengthMin]
            &tvConfidenceMin=[tvConfidenceMin]&tvCountMin=[tvCountMin]&includeIncoming=[includeIncoming]
            &includeOutgoing=[includeOutgoing]&callback=[callback]

        :param type: (optional) Atom type, see http://wiki.opencog.org/w/OpenCog_Atom_types
        :param name: (optional, string, not allowed for Link types) Atom name
        If neither type or name are provided, all atoms will be retrieved
        :param filterby: (optional, can't be combined with type or name) Allows certain predefined filters
          - The filter 'stirange' allows the additional parameters 'stimin' (required, int) and 'stimax' (optional, int)
            and returns the atoms in a given STI range
          - The filter 'attentionalfocus' (boolean) returns the atoms in the AttentionalFocus
        :param tvStrengthMin: (optional, float) Only return atoms with TruthValue strength greater than this amount
        :param tvConfidenceMin: (optional, float) Only return atoms with TruthValue confidence greater than this amount
        :param tvCountMin: (optional, float) Only return atoms with TruthValue count greater than this amount
        :param includeIncoming: (optional, boolean) Returns the conjunction of the set of atoms and their incoming sets
        :param includeOutgoing: (optional, boolean) Returns the conjunction of the set of atoms and their outgoing sets
            Useful in combination with includeIncoming.

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

        filter_by = args.get('filterby')
        sti_min = args.get('stimin')
        sti_max = args.get('stimax')

        tv_strength_min = args.get('tvStrengthMin')
        tv_confidence_min = args.get('tvConfidenceMin')
        tv_count_min = args.get('tvCountMin')

        include_incoming = args.get('includeIncoming')
        include_outgoing = args.get('includeOutgoing')

        # First, check if there is a valid filter type, and give it precedence if it exists
        valid_filter = False
        if filter_by is not None:
            if filter_by == 'stirange':
                if sti_min is not None:
                    valid_filter = True
                    atoms = self.atomspace.get_atoms_by_av(sti_min, sti_max)
                else:
                    abort(400, 'Invalid request: stirange filter requires stimin parameter')
            elif filter_by == 'attentionalfocus':
                valid_filter = True
                atoms = self.atomspace.get_atoms_in_attentional_focus()

        # If there is not a valid filter type, proceed to select by type or name
        if not valid_filter:
            if type is None and name is None:
                atoms = self.atomspace.get_atoms_by_type(types.Atom)
            elif name is None:
                atoms = self.atomspace.get_atoms_by_type(types.__dict__.get(type))
            else:
                if type is None:
                    type = 'Node'
                atoms = self.atomspace.get_atoms_by_name(t=types.__dict__.get(type), name=name)

        # Optionally, filter by TruthValue
        if tv_strength_min is not None:
            atoms = [atom for atom in atoms if atom.tv.mean >= tv_strength_min]

        if tv_confidence_min is not None:
            atoms = [atom for atom in atoms if atom.tv.confidence >= tv_confidence_min]

        if tv_count_min is not None:
            atoms = [atom for atom in atoms if atom.tv.count >= tv_count_min]

        # Optionally, include the incoming set
        if include_incoming in ['True', 'true', '1']:
            atoms = self.atomspace.include_incoming(atoms)

        # Optionally, include the outgoing set
        if include_outgoing in ['True', 'true', '1']:
            atoms = self.atomspace.include_outgoing(atoms)

        atom_list = AtomListResponse(atoms)
        json_data = {'result': atom_list.format()}
        
        # if callback function supplied, pad the JSON data (i.e. JSONP):
        if callback is not None:
            response = str(callback) + '(' + json.dumps(json_data) + ');'
            return current_app.response_class(response, mimetype='application/javascript')
        else:
            return current_app.response_class(json.dumps(json_data), mimetype='application/json')

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

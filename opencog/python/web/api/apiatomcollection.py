__author__ = 'Cosmo Harrigan'

from flask import abort, json, current_app, jsonify
from flask.ext.restful import Resource, reqparse, marshal
import opencog.cogserver
from opencog.atomspace import Atom
from mappers import *
from flask.ext.restful.utils import cors
from flask_restful_swagger import swagger

# Temporary hack
from web.api.utilities import get_atoms_by_name

# If the system doesn't have these dependencies installed, display a warning
# but allow the API to load
try:
    from graph_description import dot
except ImportError:
    print "DOT graph description format option not enabled in REST API. To " \
          "enable, install the dependencies listed here:\n" \
          "https://github.com/opencog/opencog/tree/master/opencog/python/graph_description#prerequisites"

"AtomSpace management functionality"
class AtomCollectionAPI(Resource):
    # This is because of https://github.com/twilio/flask-restful/issues/134
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def __init__(self):
        self.atom_map = global_atom_map
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('type', type=str, action='append',
                     location='args', choices=types.__dict__.keys())
        self.reqparse.add_argument('name', type=str, location='args')
        self.reqparse.add_argument('callback', type=str, location='args')
        self.reqparse.add_argument('filterby', type=str, location='args',
                                   choices=['stirange', 'attentionalfocus'])
        self.reqparse.add_argument('stimin', type=int, location='args')
        self.reqparse.add_argument('stimax', type=int, location='args')
        self.reqparse.add_argument('tvStrengthMin', type=float, location='args')
        self.reqparse.add_argument(
            'tvConfidenceMin', type=float, location='args')
        self.reqparse.add_argument('tvCountMin', type=float, location='args')
        self.reqparse.add_argument(
            'includeIncoming', type=str, location='args',
            choices=['true', 'false', 'True', 'False', '0', '1'])
        self.reqparse.add_argument(
            'includeOutgoing', type=str, location='args',
            choices=['true', 'false', 'True', 'False', '0', '1'])
        self.reqparse.add_argument(
            'dot', type=str, location='args',
            choices=['true', 'false', 'True', 'False', '0', '1'])
        self.reqparse.add_argument('limit', type=int, location='args')

        super(AtomCollectionAPI, self).__init__()
        # self.atomspace = opencog.cogserver.get_server_atomspace()

    # Set CORS headers to allow cross-origin access
    # (https://github.com/twilio/flask-restful/pull/131):
    @cors.crossdomain(origin='*')
    @swagger.operation(
	notes='''
<p>URI: <code>atoms/[id]</code>
<p>(or)
<code>atoms?type=[type]&name=[name]&filterby=[filterby]
    &tvStrengthMin=[tvStrengthMin]&tvConfidenceMin=[tvConfidenceMin]
    &tvCountMin=[tvCountMin]&includeIncoming=[includeIncoming]
    &includeOutgoing=[includeOutgoing]&limit=[limit]&callback=[callback]</code>

<p>Example:

<pre>{
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
</pre>

<p>Examples using optional predefined filters:

<dl>
  <dt>Get all atoms in the AttentionalFocus</dt>
  <dd>URI: <code>atoms?filterby=attentionalfocus</dd>
  <dt>Get all atoms in the STI range between 5 (inclusive) and 10 (inclusive)</dt>
  <dd>URI: <code>atoms?filterby=stirange&stimin=5&stimax=10</code></dd>
  <dt>Get all atoms with STI greater than or equal to 5</dt>
  <dd>URI: <code>atoms?filterby=stirange&stimin=5</code></dd>
</dl>
''',
	responseClass=Atom,
	nickname='get',
	parameters=[
	    {
		'name': 'id',
		'description': '''to specifically request an atom by handle,
		    can be combined with <code>includeIncoming</code> or <code>includeOutgoing</code> only;
		    if specified, other query parameters will have no effect) Atom handle''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'int',
		'paramType': 'path'
	    },
	    {
		'name': 'type',
		'description': '<a href="http://wiki.opencog.org/w/OpenCog_Atom_types">OpenCog Atom type</a>',
		'required': False,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'query'
	    },
	    {
		'name': 'name',
		'description': '''(not allowed for Link types). If neither
		    <code>type</code> or <code>name</code> are provided,
		    all atoms will be retrieved.''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'query'
	    },
	    {
		'name': 'filterby',
		'description': '''(can't be combined with type or name)
		    Allows certain predefined filters
		    <dl>
		      <dt>stirange</dt>
		      <dd>The filter 'stirange' allows the additional parameters 'stimin'
			(required, int) and 'stimax' (optional, int) and returns the atoms
			in a given STI range</dd>
		      <dt>attentionalfocus</dt>
		      <dd>The filter 'attentionalfocus' (boolean) returns the atoms in the
			AttentionalFocus</dd>
		    </dl>''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'stirange | attentionalfocus',
		'paramType': 'query'
	    },
	    {
		'name': 'stimin',
		'description': '''Only return atoms with STI (Short Term Importance)
		    greater than or equal to this amount (only usable with <code>filterby=stirange</code>)''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'float',
		'paramType': 'query'
	    },
	    {
		'name': 'stimax',
		'description': '''Only return atoms with STI (Short Term Importance)
		    less than or equal to this amount (only usable with <code>filterby=stirange</code>)''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'float',
		'paramType': 'query'
	    },
	    {
		'name': 'tvStrengthMin',
		'description': '''Only return atoms with
		    TruthValue strength greater than or equal to this amount''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'float',
		'paramType': 'query'
	    },
	    {
		'name': 'tvConfidenceMin',
		'description': '''Only return atoms with
		    TruthValue confidence greater than or equal to this amount''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'float',
		'paramType': 'query'
	    },
	    {
		'name': 'tvCountMin',
		'description': '''Only return atoms with
		    TruthValue count greater than or equal to this amount''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'float',
		'paramType': 'query'
	    },
	    {
		'name': 'includeIncoming',
		'description': '''Returns the conjunction of
		    the set of atoms and their incoming sets''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'boolean',
		'paramType': 'query'
	    },
	    {
		'name': 'includeOutgoing',
		'description': '''Returns the conjunction of
		    the set of atoms and their outgoing sets. Useful in combination
		    with includeIncoming.''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'boolean',
		'paramType': 'query'
	    },
	    {
		'name': 'dot',
		'description': '''Returns the atom set represented in
		    the DOT graph description language
		    (See <a href="https://github.com/opencog/opencog/blob/master/opencog/python/graph_description/README.md">opencog/python/graph_description/README.md</a> for details)''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'boolean',
		'paramType': 'query'
	    },
	    {
		'name': 'limit',
		'description': '''To specify the maximum number of atoms to be returned.
		    If the query results are greater than the number specified by
		    <code>limit</code>, then the result set list is truncated to the
		    first <code>limit</code> number of atoms.''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'int',
		'paramType': 'query'
	    },
	    {
		'name': 'callback',
		'description': '''JavaScript callback function for JSONP support''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'query'
	    }
	],
	responseMessages=[
	    {'code': 200, 'message': 'Returned list of atoms matching specified criteria'},
	    {'code': 400, 'message': 'Invalid request: stirange filter requires stimin parameter'}
	]
    )

    def get(self, id=""):
        retval = jsonify({'error':'Internal error'})
        try:
           retval = self._get(id=id)
        except Exception,e:
           retval = jsonify({'error':str(e)})
        return retval

    def _get(self, id=""):
        """
        Returns a list of atoms matching the specified criteria
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

        dot_format = args.get('dot')

        limit = args.get('limit')

        if id != "":
            try:
                atom = self.atom_map.get_atom(int(id))
                atoms = [atom]
            except IndexError:
                atoms = []
                # abort(404, 'Atom not found')
        else:
            # First, check if there is a valid filter type, and give it
            # precedence if it exists
            valid_filter = False
            if filter_by is not None:
                if filter_by == 'stirange':
                    if sti_min is not None:
                        valid_filter = True
                        atoms = self.atomspace.get_atoms_by_av(sti_min, sti_max)
                    else:
                        abort(400, 'Invalid request: stirange filter requires '
                                   'stimin parameter')
                elif filter_by == 'attentionalfocus':
                    valid_filter = True
                    atoms = self.atomspace.get_atoms_in_attentional_focus()

            # If there is not a valid filter type, proceed to select by type
            # or name
            if not valid_filter:
                if type is None and name is None:
                    atoms = self.atomspace.get_atoms_by_type(types.Atom)
                elif name is None:
                    atoms = []
                    for t in type:
                         atoms = atoms + self.atomspace.get_atoms_by_type(
                                           types.__dict__.get(t))
                else:
                    if type is None:
                        type = ['Node']
                    for t in type:
                        atoms = get_atoms_by_name(types.__dict__.get(t),
                                    name, self.atomspace)

            # Optionally, filter by TruthValue
            if tv_strength_min is not None:
                atoms = [atom for atom in atoms if atom.tv.mean >=
                                                   tv_strength_min]

            if tv_confidence_min is not None:
                atoms = [atom for atom in atoms if atom.tv.confidence >=
                                                   tv_confidence_min]

            if tv_count_min is not None:
                atoms = [atom for atom in atoms if atom.tv.count >=
                                                   tv_count_min]

        # Optionally, include the incoming set
        if include_incoming in ['True', 'true', '1']:
            atoms = self.atomspace.include_incoming(atoms)

        # Optionally, include the outgoing set
        if include_outgoing in ['True', 'true', '1']:
            atoms = self.atomspace.include_outgoing(atoms)

        # Optionally, limit number of atoms returned
        if limit is not None:
            if len(atoms) > limit:
                atoms = atoms[0:limit]

        # The default is to return the atom set as JSON atoms. Optionally, a
        # DOT return format is also supported
        if dot_format not in ['True', 'true', '1']:
            atom_list = AtomListResponse(atoms)
            # xxxxxxxxxxxx here add atoms
            json_data = {'result': atom_list.format()}

            # if callback function supplied, pad the JSON data (i.e. JSONP):
            if callback is not None:
                response = str(callback) + '(' + json.dumps(json_data) + ');'
                return current_app.response_class(
                    response, mimetype='application/javascript')
            else:
                return current_app.response_class(
                    json.dumps(json_data), mimetype='application/json')
        else:
            dot_output = dot.get_dot_representation(atoms)
            return jsonify({'result': dot_output})

    @swagger.operation(
	notes='''
Include data with the POST request providing a JSON representation of
the atom.

<p>Examples:

<p>Node:
<pre>
      {
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
</pre>

<p>Link:
<pre>
      {
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
</pre>

<p>Returns a JSON representation of an atom list containing
the atom. Example:
<pre>
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
</pre>''',
	responseClass=Atom,
	nickname='post',
	parameters=[
	    {
		'name': 'type',
		'description': '<a href="http://wiki.opencog.org/w/OpenCog_Atom_types">OpenCog Atom type</a>',
		'required': True,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'body'
	    },
	    {
		'name': 'name',
		'description': '''(required for Node types, not allowed for Link types) Atom name''',
		'required': True,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'body'
	    },
	    {
		'name': 'truthvalue',
		'description': '''<a href="http://wiki.opencog.org/w/TruthValue">TruthValue</a>, formatted as follows:
		    <dl>
		      <dt><code>type</code> (required)</dt>
		      <dd><a href="http://wiki.opencog.org/w/TruthValue">TruthValue type</a>
			(only 'simple' is currently available)</dd>
		      <dt><code>details</code> (required)</dt>
		      <dd>TruthValue parameters, formatted as follows:
			<ul>
			  <li>strength (required)</li>
			  <li>count (required)</li>
			</ul>
		      </dd>
		    </dl>''',
		'required': True,
		'allowMultiple': False,
		'dataType': 'TruthValue',
		'paramType': 'body'
	    },
	    {
		'name': 'outgoing',
		'description': '''The set of arguments of the relation, formatted as
		    <a href="http://wiki.opencog.org/w/Link#Incoming_and_Outgoing_Sets">a list of Atom handles (only valid for Links, not nodes)</a>''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'list',
		'paramType': 'body'
	    }
	],
	responseMessages=[
	    {'code': 200, 'message': 'Created specified list of atoms'},
	    {'code': 400, 'message': 'Invalid type or required parameter type missing'},
	    {'code': 500, 'message': 'Error processing request. Check your parameters'}
	]
    )
    def post(self):
        """
        Creates a new atom. If the atom already exists, it updates the atom.
        """

        # Prepare the atom data and validate it
        data = reqparse.request.get_json()

        if 'type' in data:
            if data['type'] in types.__dict__:
                type = types.__dict__.get(data['type'])
            else:
                abort(400, 'Invalid request: type \'' + type + '\' is not a '
                                                               'valid type')
        else:
            abort(400, 'Invalid request: required parameter type is missing')

        # TruthValue
        tv = ParseTruthValue.parse(data)

        # Outgoing set
        if 'outgoing' in data:
            print data
            if len(data['outgoing']) > 0:
                outgoing = [self.atom_map.get_atom(uid)
                                for uid in data['outgoing']]
        else:
            outgoing = None

        # Name
        name = data['name'] if 'name' in data else None

        # Nodes must have names
        if is_a(type, types.Node):
            if name is None:
                abort(400, 'Invalid request: node type specified and required '
                           'parameter name is missing')
        # Links can't have names
        else:
            if name is not None:
                abort(400, 'Invalid request: parameter name is not allowed for '
                           'link types')

        try:
            atom = self.atomspace.add(t=type, name=name, tv=tv, out=outgoing)
            uid = self.atom_map.get_uid(atom)
        except TypeError:
            abort(500, 'Error while processing your request. Check your '
                       'parameters.')

        dictoid = marshal(atom, atom_fields)
        dictoid['handle'] = uid
        return {'atoms': dictoid}

    @swagger.operation(
	notes='''
URI: <code>atoms/[id]</code>

<p>Include data with the PUT request providing a JSON representation of
the updated attributes.

<p>Example:

<pre>
{
  'truthvalue':
  {
    'type': 'simple',
    'details':
      {
	'strength': 0.005,
	'count': 0.8
      }
  },
'attentionvalue':
  {
    'sti': 9,
    'lti': 2,
    'vlti': True
  }
}
</pre>

<p>Returns a JSON representation of an atom list
containing the atom.

<p>Example:

<pre>
{ 'atoms':
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
}
</pre>''',
	responseClass=Atom,
	nickname='put',
	parameters=[
	    {
		'name': 'id',
		'description': '<a href="http://wiki.opencog.org/w/Atom">Atom handle</a>',
		'required': True,
		'allowMultiple': False,
		'dataType': 'int',
		'paramType': 'path'
	    },
	    {
		'name': 'truthvalue',
		'description': '''<a href="http://wiki.opencog.org/w/TruthValue">TruthValue</a>, formatted as follows:
		    <dl>
		      <dt><code>type</code> (required)</dt>
		      <dd><a href="http://wiki.opencog.org/w/TruthValue">TruthValue type</a>
			(only 'simple' is currently available)</dd>
		      <dt><code>details</code> (required)</dt>
		      <dd>TruthValue parameters, formatted as follows:
			<ul>
			  <li>strength (required)</li>
			  <li>count (required)</li>
			</ul>
		      </dd>
		    </dl>''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'TruthValue',
		'paramType': 'body'
	    },
	    {
		'name': 'attentionvalue',
		'description': '''<a href="http://wiki.opencog.org/w/AttentionValue">AttentionValue</a>, formatted as follows:
		    <dl>
		      <dt><code>sti</code> (optional)</dt>
		      <dd>Short-Term Importance</dd>
		      <dt><code>lti</code> (optional)</dt>
		      <dd>Long-Term Importance</dd>
		      <dt><code>vlti</code> (optional)</dt>
		      <dd>Very-Long-Term Importance</dd>
		    </dl>''',
		'required': False,
		'allowMultiple': False,
		'dataType': 'AttentionValue',
		'paramType': 'body'
	    }
	],
	responseMessages=[
	    {'code': 200, 'message': 'Atom truth and/or attention value updated'},
	    {'code': 400, 'message': 'Invalid type or required parameter type missing'},
	    {'code': 404, 'message': 'Atom not found'}
	]
    )
    def put(self, id):
        """
        Updates the AttentionValue (STI, LTI, VLTI) or TruthValue of an atom
        """

        # If the atom is not found in the atomspace.
        the_atom = self.atom_map.get_atom(id)
        if the_atom == None:
            abort(404, 'Atom not found')

        # Prepare the atom data
        data = reqparse.request.get_json()

        if 'truthvalue' not in data and 'attentionvalue' not in data:
            abort(400, 'Invalid request: you must include a truthvalue or '
                       'attentionvalue parameter')

        if 'truthvalue' in data:
            tv = ParseTruthValue.parse(data)
            the_atom.tv = tv

        if 'attentionvalue' in data:
            (sti, lti, vlti) = ParseAttentionValue.parse(data)
            the_atom.av = {'sti': sti, 'lti': lti, 'vlti': vlti}

        dicty = marshal(the_atom, atom_fields)
        dicty['handle'] = self.atom_map.get_uid(the_atom)
        return {'atoms': dicty}

    @swagger.operation(
	notes='''
Returns a JSON representation of the result, indicating success or failure.

<p>Example:

<pre>
{
  'result':
  {
    'handle': 2,
    'success': 'true'
  }
}
</pre>''',
	responseClass='result',
	nickname='delete',
	parameters=[
	    {
		'name': 'id',
		'description': '<a href="http://wiki.opencog.org/w/Atom">Atom handle</a>',
		'required': True,
		'allowMultiple': False,
		'dataType': 'int',
		'paramType': 'path'
	    }
	],
	responseMessages=[
	    {'code': 200, 'message': 'Deleted the atom'},
	    {'code': 404, 'message': 'Atom not found'},
	]
    )
    def delete(self, id):
        """
        Removes an atom from the AtomSpace
        """

        atom = self.atom_map.get_atom(id)
        if atom == None:
            abort(404, 'Atom not found')

        status = self.atomspace.remove(atom)
        self.atom_map.remove(atom, id)
        response = DeleteAtomResponse(id, status)
        return {'result': response.format()}

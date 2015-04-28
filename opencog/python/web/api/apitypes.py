__author__ = 'Cosmo Harrigan'

from flask import json, current_app
from flask.ext.restful import Resource, reqparse
from mappers import *
from flask.ext.restful.utils import cors
from flask_restful_swagger import swagger

class TypesAPI(Resource):
    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('callback', type=str, location='args')
        super(TypesAPI, self).__init__()

    # Set CORS headers to allow cross-origin access
    # (https://github.com/twilio/flask-restful/pull/131):
    @cors.crossdomain(origin='*')
    @swagger.operation(
	notes='''
Returns a JSON representation of a list of valid atom types

<p>Example:

<pre>
{"types": ["TrueLink", "NumberNode", "OrLink",
  "PrepositionalRelationshipNode"]}
</pre>
''',
	responseClass='response',
	nickname='get',
	parameters=[
	],
	responseMessages=[
	    {'code': 200, 'message': 'Returned list of valid atom types'},
	]
    )
    def get(self):
        """
        Returns a list of valid atom types
        """

        json_data = \
            {'types': filter(lambda x:
                             not x.startswith('__') and not x.endswith('__')
                             and not x == 'NO_TYPE', types.__dict__.keys())}

        # if callback function supplied, pad the JSON data (i.e. JSONP):
        args = self.reqparse.parse_args()
        callback = args.get('callback')
        if callback is not None:
            response = str(callback) + '(' + json.dumps(json_data) + ');'
            return current_app.response_class(
                response, mimetype='application/javascript')
        else:
            return current_app.response_class(
                json.dumps(json_data), mimetype='application/json')

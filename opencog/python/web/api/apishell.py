__author__ = 'Cosmo Harrigan'

from flask import abort, jsonify
from flask.ext.restful import Resource, reqparse
import socket
from flask_restful_swagger import swagger

COGSERVER_PORT = 17001


class ShellAPI(Resource):
    """
    Defines a barebones resource for sending shell commands to the CogServer
    """

    # This is because of https://github.com/twilio/flask-restful/issues/134
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('command', type=str, location='args')

        super(ShellAPI, self).__init__()

    @swagger.operation(
	notes='''
Include a JSON object with the POST request containing the command
in a field named "command"

<p>Examples:

<pre>
{'command': 'agents-step'}
{'command': 'agents-step opencog::SimpleImportanceDiffusionAgent'}
</pre>''',
	responseClass='response',
	nickname='post',
	parameters=[
	    {
		'name': 'command',
		'description': 'OpenCog Shell command',
		'required': True,
		'allowMultiple': False,
		'dataType': 'string',
		'paramType': 'body'
	    }
	],
	responseMessages=[
	    {'code': 200, 'message': 'OpenCog Shell command executed successfully'},
	    {'code': 400, 'message': 'Invalid request: Required parameter command missing'}
	]
    )
    def post(self):
        """
        Send a shell command to the cogserver
        """

        # Setup socket to communicate with OpenCog CogServer
        try:
            connection = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            connection.connect(('localhost', COGSERVER_PORT))
        except socket.error as msg:
            print msg

        # Validate, parse and send the command
        data = reqparse.request.get_json()
        if 'command' in data:
            connection.send(data['command'])
        else:
            connection.close()
            abort(400,
                  'Invalid request: required parameter command is missing')

        connection.close()

        return jsonify({'status': 'success'})

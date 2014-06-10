__author__ = 'Cosmo Harrigan'

from flask import abort, jsonify
from flask.ext.restful import Resource, reqparse
from opencog.scheme_wrapper import scheme_eval, __init__

COGSERVER_PORT = 17001


class SchemeAPI(Resource):
    """
    Defines an interface for issuing commands to and receiving responses from
    the OpenCog Scheme interpreter
    """

    # This is because of https://github.com/twilio/flask-restful/issues/134
    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('command', type=str, location='args')

        super(SchemeAPI, self).__init__()

    def post(self):
        """
        Send a command to the Scheme interpreter
        Uri: scheme

        Include a JSON object with the POST request containing the command
        in a field named "command"

        Example command:

        {'command': '(cog-set-af-boundary! 100)'}

        Returns:

        A JSON object containing the Scheme-formatted result of the command in
        a field named "response".

        Example response:

        {'response': '100\n'}

        Note that in this API, the request is processed synchronously. It
        blocks until the request has finished.

        This functionality is implemented as a POST method because it can
        cause side-effects.
        """

        # Validate, parse and send the command
        data = reqparse.request.get_json()
        if 'command' in data:
            response = scheme_eval(self.atomspace, data['command'])
        else:
            abort(400,
                  'Invalid request: required parameter command is missing')

        return jsonify({'response': response})

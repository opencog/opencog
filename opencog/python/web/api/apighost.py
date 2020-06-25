__author__ = 'xabush_semrie and aman'

from flask_restful import Resource, reqparse
from opencog.scheme_wrapper import scheme_eval
from flask import abort, jsonify



class GhostApi(Resource):

    @classmethod
    def new(cls, atomspace):
        cls.atomspace = atomspace
        return cls

    def __init__(self):
        self.reqparse = reqparse.RequestParser()
        self.reqparse.add_argument('query', type=str, location='args')
        super(GhostApi, self).__init__()


    def post(self):
        data = reqparse.request.get_json()
        print(data)
        if 'query' in data:
            query = data['query']
            print(query)
            action = "(test-ghost \"{}\")".format(query)
            scheme_eval(self.atomspace, action)
            response = scheme_eval(self.atomspace, '(get-result)')
            print(response)
        else:
            abort(400,
                  'Invalid request: required parameter command is missing')

        return jsonify({'response': response})



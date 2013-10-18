__author__ = 'Cosmo Harrigan'

from flask import Flask, abort
from flask.ext.restful import Api
from apiatom import *
from apiatomcollection import *


class RESTAPI(object):
    """
    REST API for OpenCog

    Implemented using the Flask micro-framework and Flask-RESTful extension

    Default endpoint: http://127.0.0.1:5000/api/v1.0/
    (Replace 127.0.0.1 with the IP address of the server)

    Example request: http://127.0.0.1:5000/api/v1.0/atoms?type=ConceptNode

    See: tests/python/test_restapi.py for extensive examples, and review
    the method definitions in each resource for request/response specifications.

    If accessing the API from Python, you may find it convenient to install
    the 'requests' module to easily perform requests and responses.
    """

    def __init__(self, atomspace):
        self.atomspace = atomspace

        # Initialize the web server and set the routing
        self.app = Flask(__name__, static_url_path="")
        self.api = Api(self.app)
        atom_api = AtomAPI.new(self.atomspace)
        atom_collection_api = AtomCollectionAPI.new(self.atomspace)
        self.api.add_resource(atom_collection_api, '/api/v1.0/atoms', endpoint='atoms')
        self.api.add_resource(atom_api, '/api/v1.0/atoms/<int:id>', endpoint='atom')

    def run(self):
        self.app.run(debug=False)

    def test(self):
        return self.app.test_client()



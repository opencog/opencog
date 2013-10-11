"""
REST API for OpenCog

Implemented using the Flask micro-framework and flask-restful extension
"""

__author__ = 'Cosmo Harrigan'

from flask import Flask, abort
from flask.ext.restful import Api
from apiatom import *
from apiatomcollection import *


class RESTApi(object):
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
        self.app.run(debug=True)

    def test(self):
        return self.app.test_client()



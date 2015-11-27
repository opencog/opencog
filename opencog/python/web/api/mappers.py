"""
Utilities used in the REST API for OpenCog

Defines mapping between object attributes and API requests and responses
Validates input data, formats output data
Used by the classes in: apiatomcollection.py
"""

__author__ = 'Cosmo Harrigan'

from flask import abort
from flask.ext.restful import fields, marshal
from opencog.atomspace import *


# TruthValue helpers
class ParseTruthValue(object):
    @staticmethod
    def parse(data):
        if 'truthvalue' in data:
            if 'type' in data['truthvalue']:
                tv_type = data['truthvalue']['type']
            else:
                abort(400, 'Invalid request: truthvalue object requires a '
                           'type parameter')
        else:
            abort(400, 'Invalid request: required parameter truthvalue '
                       'is missing')

        # @todo: Cython bindings implementation does not provide support for
        # other TruthValue types yet
        # (see: opencog\cython\opencog\atomspace_details.pyx,
        #       opencog\cython\opencog\atomspace.pxd)
        if tv_type != 'simple':
            if tv_type in ['composite', 'count', 'indefinite']:
                # @todo: check error type
                abort(400, 'Invalid request: truthvalue type \'' +
                           tv_type + '\' is not supported')
            else:
                # @todo: check error type
                abort(400, 'Invalid request: type \'' + tv_type +
                           '\' is not a valid truthvalue type')

        if 'details' in data['truthvalue']:
            if 'strength' in data['truthvalue']['details'] \
                and 'count' in data['truthvalue']['details']:
                tv = TruthValue(data['truthvalue']['details']['strength'],
                                TruthValue.count_to_confidence(data['truthvalue']['details']['count']))
            else:
                abort(400, 'Invalid request: truthvalue details object '
                           'requires both a strength and count parameter')
        else:
            abort(400, 'Invalid request: truthvalue object requires a '
                       'details parameter')

        return tv


class FormatTruthValue(fields.Raw):
    def format(self, value):
        return {
            'type': 'simple',
            'details': marshal(value, tv_fields)
        }

tv_fields = {
    'strength': fields.Float(attribute='mean'),
    'confidence': fields.Float(attribute='confidence'),
    'count': fields.Float(attribute='count')
}


# AttentionValue helpers
class ParseAttentionValue(object):
    @staticmethod
    def parse(data):
        av = data['attentionvalue']
        sti = av['sti'] if 'sti' in av else None
        lti = av['lti'] if 'lti' in av else None
        vlti = av['vlti'] if 'vlti' in av else None

        return sti, lti, vlti

av_fields = {
    'sti': fields.Integer(attribute='sti'),
    'lti': fields.Integer(attribute='lti'),
    'vlti': fields.Boolean(attribute='vlti')
}


# Atom helpers
class FormatHandleValue(fields.Raw):
    def format(self, value):
        return value.value()


class FormatHandleList(fields.Raw):
    def format(self, values):
        return [elem.h.value() for elem in values]


class AtomListResponse(object):
    def __init__(self, atoms):
        self.atoms = atoms

    def format(self):
        return {
            # @todo: Add pagination (http://flask.pocoo.org/snippets/44/)
            'complete': True,
            'skipped': 0,
            'total': len(self.atoms),
            'atoms': marshal(self.atoms, atom_fields)
        }


class DeleteAtomResponse(object):
    def __init__(self, handle, status):
        self.handle = handle
        self.status = status

    def format(self):
        return {
            'handle': self.handle,
            'success': self.status
        }

atom_fields = {
    'handle': FormatHandleValue(attribute='h'),
    'type': fields.String(attribute='type_name'),
    'name': fields.String,
    'outgoing': FormatHandleList(attribute='out'),
    'incoming': FormatHandleList(attribute='incoming'),
    'truthvalue': FormatTruthValue(attribute='tv'),
    'attentionvalue': fields.Nested(av_fields, attribute='av')
}

__author__ = 'raminbarati'

class Modification(object):
    ADD_NODE = 0
    REMOVE_NODE = 1
    ADD_LINK = 2
    REMOVE_LINK = 3

    def __init__(self, type, data):
        self.data = data
        self.type = type

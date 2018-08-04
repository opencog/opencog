

from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types

__author__ = 'Hujie Wang'

'''
DumpAgent is used to dump atomspace content into a file for testing purposes.
'''

class dumpAgent(MindAgent):
    
    def run(self, atomspace):
        with open('/tmp/log.txt', 'w') as logfile:
            all_atoms = atomspace.get_atoms_by_type(t=types.Atom)
            for atom in all_atoms:
                print(atom, file=logfile)

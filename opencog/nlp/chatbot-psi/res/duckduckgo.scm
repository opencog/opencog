(use-modules (opencog) (opencog python))

(python-eval "
from opencog.atomspace import AtomSpace, types, TruthValue

import urllib2
import json

atomspace = ''

def set_atomspace(atsp):
    global atomspace
    atomspace = atsp
    return TruthValue(1, 1)

def call_duckduckgo(query):
    global atomspace
    response = urllib2.urlopen('http://api.duckduckgo.com/?q=' + query.name + '&format=json').read()
    result = json.loads(response)
    atomspace.add_node(types.Node, result['AbstractText'])
    return TruthValue(1, 1)
")

(python-call-with-as "set_atomspace" (cog-atomspace))
(cog-evaluate! (Evaluation (GroundedPredicate "py: call_duckduckgo") (List (Node "OpenCog"))))

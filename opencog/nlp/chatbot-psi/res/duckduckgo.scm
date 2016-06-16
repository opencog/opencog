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

# TODO: Attribution!
def call_duckduckgo(query):
    global atomspace

    # Update the state
    answer_anchor = atomspace.add_node(types.AnchorNode, 'DuckDuckGoAnswers')
    search_anchor = atomspace.add_node(types.AnchorNode, 'DuckDuckGoSearch')
    search_started = atomspace.add_node(types.ConceptNode, 'SearchStarted')
    atomspace.add_link(types.StateLink, [search_anchor, search_started])

    # Send the query
    response = urllib2.urlopen('http://api.duckduckgo.com/?q=' + query.name + '&format=json').read()
    result = json.loads(response)
    abstract_text = result['AbstractText']

    if abstract_text:
        ans = atomspace.add_node(types.Node, abstract_text)
        atomspace.add_link(types.StateLink, [answer_anchor, ans])
    else:
        no_result = atomspace.add_node(types.Node, 'NoResult')
        atomspace.add_link(types.StateLink, [answer_anchor, no_result])

    return TruthValue(1, 1)
")

(python-call-with-as "set_atomspace" (cog-atomspace))
(cog-evaluate! (Evaluation (GroundedPredicate "py: call_duckduckgo") (List (Node "OpenCog"))))

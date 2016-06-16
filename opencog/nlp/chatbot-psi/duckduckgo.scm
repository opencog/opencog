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
def call_duckduckgo(qq):
    global atomspace

    # Anchor for the result
    answer_anchor = atomspace.add_node(types.AnchorNode, 'DuckDuckGoAnswers')

    # Avoid HTTP Error 400: Bad Request
    query = qq.name.replace(' ', '+')

    # Send the query
    response = urllib2.urlopen('http://api.duckduckgo.com/?q=' + query + '&format=json').read()
    result = json.loads(response)
    abstract_text = result['AbstractText']

    if abstract_text:
        ans = atomspace.add_node(types.Node, abstract_text)
        atomspace.add_link(types.StateLink, [answer_anchor, ans])
    else:
        no_result = atomspace.add_node(types.ConceptNode, 'NoResult')
        atomspace.add_link(types.StateLink, [answer_anchor, no_result])

    return TruthValue(1, 1)
")

(define (ask-duckduckgo)
    (State duckduckgo-search search-started)

    ; TODO: We may want to actually nlp-parse the answer, but a typical answer
    ; of this type seems to be very long (a paragraph), split into sentences
    ; and then parse?
    (add-thread (begin-thread
        (python-call-with-as "set_atomspace" (cog-atomspace))
        (cog-evaluate! (Evaluation (GroundedPredicate "py: call_duckduckgo") (List (get-input-text-node))))
    ))
)

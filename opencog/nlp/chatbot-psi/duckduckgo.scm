; This will try to get results from DuckDuckGo
; https://duckduckgo.com/

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
    answer_anchor = atomspace.add_node(types.AnchorNode, 'Chatbot: DuckDuckGoAnswers')

    # Avoid HTTP Error 400: Bad Request
    query = qq.name.replace(' ', '+')

    # Send the query
    response = urllib2.urlopen('http://api.duckduckgo.com/?q=' + query + '&format=json').read()
    result = json.loads(response)
    abstract_text = result['AbstractText']

    if abstract_text:
        word_nodes = []
        words = abstract_text.split(' ')
        for word in words:
            word_nodes.append(atomspace.add_node(types.WordNode, word))
        ans = atomspace.add_link(types.ListLink, word_nodes)
        atomspace.add_link(types.StateLink, [answer_anchor, ans])
    else:
        no_result = atomspace.add_node(types.ConceptNode, 'Chatbot: NoResult')
        atomspace.add_link(types.StateLink, [answer_anchor, no_result])

    return TruthValue(1, 1)
")

(define (ask-duckduckgo)
    (State duckduckgo-search search-started)

    ; TODO: We may want to actually nlp-parse the answer, but a typical answer
    ; of this type seems to be very long (a paragraph), split into sentences
    ; and then parse?
    (begin-thread
        (python-call-with-as "set_atomspace" (cog-atomspace))
        (cog-evaluate! (Evaluation (GroundedPredicate "py: call_duckduckgo") (List (get-input-text-node))))
        (State duckduckgo-search search-finished)
    )
)

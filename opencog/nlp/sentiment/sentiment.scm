;
; sentiment.scm
;
; Sentiment analysis
;
; -----------------------------------------------------------------------

(use-modules (opencog) (opencog python) (opencog exec))

(python-eval "
from opencog.atomspace import AtomSpace, types, TruthValue

have_sentiment_analysis = True
try:
      import basic_sentiment_analysis
except ImportError:
      have_sentiment_analysis = False

atomspace = ''

def set_atomspace(atsp):
      global atomspace
      atomspace = atsp
      return TruthValue(1, 1)

def call_sentiment_parse(text_node, sent_node):
      global atomspace
      global have_sentiment_analysis

      if not have_sentiment_analysis:
          return TruthValue(1, 1)

      sentiment_score = basic_sentiment_analysis.sentiment_parse(text_node.name)
      if sentiment_score > 0:
          positive_node = atomspace.add_node(types.ConceptNode, 'Positive')
          atomspace.add_link(types.InheritanceLink, [sent_node, positive_node])
      elif sentiment_score < 0:
          negative_node = atomspace.add_node(types.ConceptNode, 'Negative')
          atomspace.add_link(types.InheritanceLink, [sent_node, negative_node])
      else:
          neutral_node = atomspace.add_node(types.ConceptNode, 'Neutral')
          atomspace.add_link(types.InheritanceLink, [sent_node, neutral_node])

      return TruthValue(1, 1)
")

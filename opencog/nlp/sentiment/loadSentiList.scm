; load the word list from the simplified sentiWordNet filename

(use-modules (ice-9 threads)  ; needed for par-map
             (srfi srfi-1)
             (opencog)
             (opencog python)
             (opencog exec)
             (opencog ure)
             (opencog nlp)
             (opencog nlp oc)
             (opencog nlp fuzzy)
             (opencog nlp microplanning)
             (opencog nlp relex2logic))

; -----------------------------------------------------------------------

(python-eval "
from opencog.atomspace import AtomSpace, types, TruthValue
import sys
import os

class SentimentWord:
    word = ''
    posScore = 0.0
    negScore = 0.0


def loadSentiWords (filename):
    words = []
    with open(filename, 'r') as f:
        for line in f:
            sentiWord = SentimentWord()
            t = line.split('\t\t')
            sentiWord.word = t[0]
            sentiWord.posScore = float(t[1])
            sentiWord.negScore = float(t[2])
            words.append(sentiWord)
    return words

def load_sentiment_list(atomspace):
      configpath = '/usr/local/etc'
      path = os.path.join(configpath, 'opencog/dicts/')
      sentiWordList = loadSentiWords(os.path.join(path, 'sentiWordNet.txt'))
      for sentiWord in sentiWordList:
            word_node = atomspace.add_node(types.ConceptNode, sentiWord.word)
            if sentiWord.posScore > 0:
                positive_node = atomspace.add_node(types.ConceptNode, 'Positive')
                positive_link = atomspace.add_link(types.InheritanceLink, [word_node, positive_node])
                # For temporary use, the strength is set as : s = c + weight * (1 - c), where c = 0.5, weight is the PosScore/NegScore from sentiWordNet
                postive_strength = 0.5 + sentiWord.posScore * 0.5
                positive_link.tv = TruthValue(postive_strength, 0.9)
            if sentiWord.negScore > 0:
                negative_node = atomspace.add_node(types.ConceptNode, 'Negative')
                negative_link = atomspace.add_link(types.InheritanceLink, [word_node, negative_node])
                negative_strength = 0.5 + sentiWord.negScore * 0.5
                negative_link.tv = TruthValue(negative_strength, 0.9)

      return TruthValue(1, 1)
")

(python-call-with-as "load_sentiment_list" (cog-atomspace))

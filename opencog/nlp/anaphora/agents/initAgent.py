from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types
from opencog.scheme_wrapper import load_scm,scheme_eval,scheme_eval_h, __init__

__author__ = 'hujie'

class initAgent(MindAgent):
    def run(self, atomspace):
        self.atomspace=atomspace
        data=["opencog/nlp/anaphora/rules/getChildren.scm",
              "opencog/nlp/anaphora/rules/getNumberNode.scm",
              "opencog/nlp/anaphora/rules/getRoots.scm",
              "opencog/nlp/anaphora/rules/getPronouns.scm",
              "opencog/nlp/anaphora/rules/propose.scm",
              "opencog/nlp/anaphora/rules/getResults.scm",
              "opencog/nlp/anaphora/rules/getAllNumberNodes.scm",

              "opencog/nlp/anaphora/rules/filtersGenerator.scm",

              "opencog/nlp/anaphora/rules/filters/filter-#1.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#2.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#3.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#4.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#5.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#6.scm",
              ]
        for item in data:
            load_scm(atomspace, item)
from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types
from opencog.scheme_wrapper import load_scm,scheme_eval,scheme_eval_h, __init__
from anaphora.python_classes.BindLinkExecution import BindLinkExecution

__author__ = 'hujie'

class initAgent(MindAgent):
    def __init__(self):
        self.numOfFilters=4

    def bindLinkExe(self,anchorNode, target, command,resultNode,atomType):
        exe=BindLinkExecution(self.atomspace,anchorNode, target, command,resultNode,atomType)
        exe.execution()
        rv=exe.returnResult()
        exe.clear()
        return rv

    def generateCommand(self,index):
        return '(define filter-instance-#'+str(index)+' (filterGenerator filter-#'+str(index)+'))'

    def initFilters(self):
        for i in range(1,self.numOfFilters):
            command=self.generateCommand(i)
            self.bindLinkExe(None,None,command,None,None)

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
              ]
        for item in data:
            load_scm(atomspace, item)

        self.initFilters()
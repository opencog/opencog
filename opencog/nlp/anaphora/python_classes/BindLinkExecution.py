__author__ = 'hujie'

from opencog.atomspace import types
from opencog.scheme_wrapper import load_scm,scheme_eval,scheme_eval_h, __init__
from opencog.atomspace import types, AtomSpace, TruthValue

class BindLinkExecution():
    def __init__(self,atomspace,anchorNode, target, command,resultNode,atomType):
        self.atomspace=atomspace
        self.anchorNode=anchorNode
        self.target=target
        self.command=command
        self.resultNode=resultNode
        self.atomType=atomType
    def execution(self):
        if self.anchorNode != None and self.target != None:
            self.tmpLink=self.atomspace.add_link(types.ListLink, [self.anchorNode, self.target], TruthValue(1.0, 100))
        else:
            self.tmpLink=None
        response = scheme_eval(self.atomspace, self.command)
        #time.sleep(0.5)
        a=3

    def returnResult(self):
        if self.resultNode==None:
            return
        rv=[]
        listOfLinks=self.resultNode.incoming
        for link in listOfLinks:
            atom=(link.out)[1]
            if atom.type==self.atomType:
                rv.append(atom)

        for link in listOfLinks:
            self.atomspace.remove(link)
        return rv

    def clear(self):
        if self.tmpLink!=None:
            self.atomspace.remove(self.tmpLink)

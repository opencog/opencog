__author__ = 'hujie'

from opencog.cogserver import MindAgent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import scheme_eval_h,scheme_eval, __init__
import Queue

class BindLinkExecution():
    def __init__(self,atomspace,anchorNode, target, command,resultNode):
        self.atomspace=atomspace
        self.anchorNode=anchorNode
        self.target=target
        self.command=command
        self.resultNode=resultNode

    def execution(self):
        if self.anchorNode != None and self.target != None:
            self.tmpLink=self.atomspace.add_link(types.ListLink, [self.anchorNode, self.target], TruthValue(1.0, 100))
        else:
            self.tmpLink=None
        response = scheme_eval_h(self.atomspace, self.command)

    def returnResult(self):
        if self.resultNode==None:
            return
        rv=[]
        listOfLinks=self.resultNode.incoming
        for link in listOfLinks:
            rv.append((link.out)[1])
        return rv

    def clear(self):
        if self.tmpLink!=None:
            self.atomspace.remove(self.tmpLink)
        name=self.resultNode.name
        self.atomspace.remove(self.resultNode,True)
        self.atomspace.add_node(types.AnchorNode, name, TruthValue(1.0, 100))


class HobbsAgent(MindAgent):
    def __init__(self):
        self.checked={}

    def bindLinkExe(self,anchorNode, target, command,resultNode):
        bindLinkExe=BindLinkExecution(self.atomspace,anchorNode, target, command,resultNode)
        bindLinkExe.execution()
        rv=bindLinkExe.returnResult()
        bindLinkExe.clear()
        return rv

    def StringToNumber(self,str):
        return int(str)

    def getWordNumber(self,item):
        rv=self.bindLinkExe(self.currentTarget,item,'(cog-bind getNumberNode)',self.currentResult)
        return self.StringToNumber(rv[0].name)

    def sortChildren(self,list):
        sorted(list,key=self.getWordNUmber)

    def getChildren(self,node):
        rv=self.bindLinkExe(self.currentTarget,node,'(cog-bind getChildren)',self.currentResult)
        self.sortChildren(rv)
        return rv

    def propose(self,node):
        self.bindLinkExe(self.currentProposal,node,'(cog-bind propose)',None)

    def bfs(self,node):
        q=Queue()
        q.put(node)
        while not q.empty():
            front=q.get()
            self.propose(front)
            childrens=self.getChildren(front)
            for node in childrens:
                if not self.Checked(node):
                    self.checked[node.name]=True
                    q.put(node)

    def getPronouns(self):
        return self.bindLinkExe(None,None,'(cog-bind pronoun-finder)',self.unresolvedReferences)

    def getRoots(self):
        return self.bindLinkExe(None,None,'(cog-bind-crisp getRoots)',self.currentResult)

    def getRootOfNode(self,target):
        '''
        rv=self.bindLinkExe(self.currentTarget,target,'(cog-bind getRootOfNode)',self.currentResult)
        maximum=-1
        maxTarget=None
        for node in rv:
            number=self.getWordNumber(node)
            if number>maximum:
                maximum=number
                maxTarget=node
        return maxTarget
        '''
        '''
        Naive approach, but works
        '''
        return self.roots[len(self.roots)-1]

    def  previousRootExist(self,root):
        return not self.roots[0]==root

    def getPrevious(self,root):
        maximum=-1
        maxTarget=None
        for item in self.roots:
            number=self.getWordNumber(item)
            if number>maximum and number<self.pronounNumber:
                maximum=number
                maxTarget=item
        return maxTarget

    def initilization(self,atomspace):
        self.atomspace = atomspace

        self.currentPronounNode = atomspace.add_node(types.AnchorNode, 'CurrentPronoun', TruthValue(1.0, 100))
        self.currentTarget = atomspace.add_node(types.AnchorNode, 'CurrentTarget', TruthValue(1.0, 100))
        self.currentResult = atomspace.add_node(types.AnchorNode, 'CurrentResult', TruthValue(1.0, 100))
        self.currentProposal = atomspace.add_node(types.AnchorNode, 'CurrentProposal', TruthValue(1.0, 100))
        self.unresolvedReferences=atomspace.add_node(types.AnchorNode, 'Recent Unresolved references', TruthValue(1.0, 100))
        self.pronounNumber = -1




        self.pronouns = self.getPronouns()
        self.roots = self.getRoots()
    def run(self, atomspace):
        self.initilization(atomspace)

        for pronoun in self.pronouns:
            self.checked=[]
            self.pronounNumber=self.getWordNumber(pronoun)
            tmpLink=self.atomspace.add_link(types.ListLink, [self.currentPronounNode, pronoun], TruthValue(1.0, 100))
            root=self.getRoot(pronoun)
            while True:
                self.bfs(root)
                if self.previousRootExist(root):
                    root=self.getPrevious(root)
                else:
                    break
            self.atomspace.remove(tmpLink)

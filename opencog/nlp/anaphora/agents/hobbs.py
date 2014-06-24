
from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import scheme_eval_h,scheme_eval, __init__
import Queue
import time
__author__ = 'hujie'

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


class HobbsAgent(MindAgent):
    def __init__(self):
        self.checked=dict()
        self.wordNumber=dict()
        self.atomspace = None

        self.currentPronounNode = None
        self.currentTarget = None
        self.currentResult = None
        self.currentProposal = None
        self.unresolvedReferences=None
        self.pronounNumber = None

        self.pronouns = None
        self.roots = None

    def bindLinkExe(self,anchorNode, target, command,resultNode,atomType):
        exe=BindLinkExecution(self.atomspace,anchorNode, target, command,resultNode,atomType)
        exe.execution()
        rv=exe.returnResult()
        exe.clear()
        return rv

    def StringToNumber(self,str):
        return int(str)

    def getWordNumber(self,node):
        '''
        Some threading bugs, cannot be solved right now

        rv=self.bindLinkExe(self.currentTarget, node ,'(cog-bind getNumberNode)',self.currentResult,types.NumberNode)
        if len(rv)>0:
            return self.StringToNumber(rv[0].name)
        else:
            print(node)
            return 0
        '''
        if node==None:
            print("found you")
            return 0
        print("word:")
        print(node.name)
        print("number:")
        print(self.wordNumber[node.name])
        return self.wordNumber[node.name]
    def sortNodes(self,list):
        sorted(list,key=self.getWordNumber,reverse=True)
        print("sorted:")
        print(list)

    def getChildren(self,node):
        rv=self.bindLinkExe(self.currentTarget,node,'(cog-bind getChildren)',self.currentResult,types.WordInstanceNode)
        self.sortNodes(rv)
        return rv

    def propose(self,node):
        self.bindLinkExe(self.currentProposal,node,'(cog-bind propose)',None,None)

    def Checked(self,node):
        if node.name in self.checked:
            return True
        self.checked[node.name]=True
        return False

    def bfs(self,node):
        if node==None:
            print("found you bfs")
            return
        q=Queue.Queue()
        q.put(node)
        while not q.empty():
            front=q.get()
            self.propose(front)
            children=self.getChildren(front)
            if len(children)>0:
                for node in children:
                    if not self.Checked(node):
                        q.put(node)

    def getPronouns(self):
        return self.bindLinkExe(None,None,'(cog-bind pronoun-finder)',self.unresolvedReferences,types.WordInstanceNode)

    def getRoots(self):
        rv= self.bindLinkExe(None,None,'(cog-bind-crisp getRoots)',self.currentResult,types.WordInstanceNode)
        self.sortNodes(rv)
        return rv

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
        return not self.roots[0].name==root.name

    def getPrevious(self,root):

        rootNumber=self.getWordNumber(root)
        for root in reversed(self.roots):
            number=self.getWordNumber(root)
            if number<rootNumber:
                return root
        print("Impossible")
    def getAllNumberNodes(self):
        rv= self.bindLinkExe(None,None,'(cog-bind getAllNumberNodes)',self.currentResult,types.WordSequenceLink)
        for link in rv:
            out=link.out
            if out[0].type==types.WordInstanceNode:
                self.wordNumber[out[0].name]=self.StringToNumber(out[1].name)

    def initilization(self,atomspace):
        self.atomspace = atomspace

        self.currentPronounNode = atomspace.add_node(types.AnchorNode, 'CurrentPronoun', TruthValue(1.0, 100))
        self.currentTarget = atomspace.add_node(types.AnchorNode, 'CurrentTarget', TruthValue(1.0, 100))
        self.currentResult = atomspace.add_node(types.AnchorNode, 'CurrentResult', TruthValue(1.0, 100))
        self.currentProposal = atomspace.add_node(types.AnchorNode, 'CurrentProposal', TruthValue(1.0, 100))
        self.unresolvedReferences=atomspace.add_node(types.AnchorNode, 'Recent Unresolved references', TruthValue(1.0, 100))
        self.currentResolutionNode=atomspace.add_node(types.AnchorNode, 'CurrentResolution', TruthValue(1.0, 100))
        self.currentResolutionLink_proposal=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentProposal], TruthValue(1.0, 100))
        self.currentResolutionLink_pronoun=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentPronounNode], TruthValue(1.0, 100))
        self.pronounNumber = -1

        self.getAllNumberNodes()
        self.pronouns = self.getPronouns()
        self.roots = self.getRoots()


    def printResults(self):
        rv = self.bindLinkExe(None,None,'(cog-bind getResults)',self.currentResult,types.ReferenceLink)

        with open('/tmp/results.txt', 'w') as logfile:
            for atom in rv:
                print(atom)
                print(atom, file=logfile)

    def run(self, atomspace):
        self.initilization(atomspace)

        for pronoun in self.pronouns:
            self.checked.clear()
            self.pronounNumber=self.getWordNumber(pronoun)
            tmpLink=self.atomspace.add_link(types.ListLink, [self.currentPronounNode, pronoun], TruthValue(1.0, 100))
            root=self.getRootOfNode(pronoun)
            print("Resolving...........")
            print(pronoun)

            while True:
                print("in")
                if root==None:
                    print("found you while")
                    break
                self.bfs(root)
                if self.previousRootExist(root):
                    root=self.getPrevious(root)
                else:
                    break
            self.atomspace.remove(tmpLink)

        self.printResults()

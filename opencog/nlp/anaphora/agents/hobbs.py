__author__ = 'hujie'

from opencog.cogserver import MindAgent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import scheme_eval_h, __init__
import Queue

class HobbsAgent(MindAgent):
    def __init__(self):
        self.checked={}

    def StringToNumber(self,str):
        return int(str)

    def getWordNumber(self,item):
        tmpLink=self.atomspace.add_link(types.ListLink, [self.currentTarget, item], TruthValue(1.0, 100))
        response = scheme_eval_h(self.atomspace, '(cog-bind getNumberNode)')
        AnchorNode=self.atomspace.get_atoms_by_name(types.AnchorNode,"CurrentResult")
        listOfLinks=AnchorNode.incoming
        link=listOfLinks[0]
        return self.StringToNumber((link.out)[1].name)

    def sortChildren(self,list):
        sorted(list,key=self.getWordNUmber)

    def getChildrens(self,node):
        rv=[]
        dict={}

        tmpLink=self.atomspace.add_link(types.ListLink, [self.targetAnchorNode, node], TruthValue(1.0, 100))
        response = scheme_eval_h(self.atomspace, '(cog-bind getChildrens)')
        self.atomspace.remove(tmpLink)
        childrenAnchorNode=self.atomspace.get_atoms_by_name(types.AnchorNode,"ChildrenAnchorNode")
        listOfLinks=childrenAnchorNode.incoming
        for link in listOfLinks:
            outgoing=link.out
            rv.append((outgoing[1])
        self.sortChildren(rv)
        return rv


    def bfs(self,node):
        q=Queue()
        q.put(node)
        while not q.empty():
            front=q.get()
            childrens=self.getChildrens(front)
            for node in childrens:
                if not self.Checked(node):
                    self.checked[node.name]=True
                    q.put(node)

    def getPronouns(self):
        rv=[]
        response = scheme_eval_h(self.atomspace, '(cog-bind pronoun-finder)')
        UnresolvedPronounsAnchorNode=self.atomspace.get_atoms_by_name(types.AnchorNode,"Recent Unresolved references")
        listOfLinks=UnresolvedPronounsAnchorNode.incoming
        for link in listOfLinks:
            rv.append((link.out)[1])
        return rv

    def getRoots(self):
        rv=[]
        response = scheme_eval_h(self.atomspace, '(cog-bind getRoots)')
        RootsAnchorNode=self.atomspace.get_atoms_by_name(types.AnchorNode,"Roots")
        listOfLinks=RootsAnchorNode.incoming
        for link in listOfLinks:
            rv.append((link.out)[1])
        return rv

    def getRootOfNode(self,target):
        rv=[]

        tmpLink=self.atomspace.add_link(types.ListLink, [self.currentTarget, target], TruthValue(1.0, 100))
        response = scheme_eval_h(self.atomspace, '(cog-bind getRootOfNode)')

        listOfLinks=self.currentResult.incoming
        maximum=-1
        maxTarget=None
        for link in listOfLinks:
            number=self.getWordNumber((link.out)[1])
            if number>maximum:
                maximum=number
                maxTarget=(link.out)[1]
        return maxTarget

    def  previousRootExist(self,root):
        return self.roots[0]==root

    def getPrevious(self,root):
        maximum=-1
        maxTarget=None
        for item in self.roots:
            number=self.getWordNumber(item)
            if number>maximum and number<self.pronounNumber:
                maximum=number
                maxTarget=item
        return maxTarget

    def run(self, atomspace):
        self.atomspace = atomspace
        self.pronouns = self.getPronouns()
        self.roots = self.getRoots()
        self.currentPronounNode = atomspace.add_node(types.AnchorNode, 'CurrentPronoun', TruthValue(1.0, 100))
        self.currentTarget = atomspace.add_node(types.AnchorNode, 'CurrentTarget', TruthValue(1.0, 100))
        self.currentResult = atomspace.add_node(types.AnchorNode, 'CurrentResult', TruthValue(1.0, 100))
        self.pronounNumber = -1

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

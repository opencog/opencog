
from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm,scheme_eval_h,scheme_eval, __init__
import Queue
import time

__author__ = 'Hujie Wang'


class BindLinkExecution():

    '''
    Executes a (cog-bind xxx) command and return the results of it
    '''

    def __init__(self,atomspace,anchorNode, target, command,resultNode,atomType):

        '''
        Stores necessary information
        '''

        self.atomspace=atomspace
        self.anchorNode=anchorNode
        self.target=target
        self.command=command
        self.resultNode=resultNode
        self.atomType=atomType

    def execution(self):

        '''
        First binds the "anchorNode" with the "target" if "anchorNode" exists, then executes scheme command "command"
        '''

        if self.anchorNode != None and self.target != None:
            self.tmpLink=self.atomspace.add_link(types.ListLink, [self.anchorNode, self.target], TruthValue(1.0, 100))
        else:
            self.tmpLink=None
        response = scheme_eval(self.atomspace, self.command)
        d=3;

    def returnResult(self):

        '''
        Returns list of atoms resulted in previous execution of a scheme command
        It only returns atoms which match the type of "self.atomType"
        '''

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

        '''
        Cleans up the Link between the "anchorNode" and the "target".
        '''

        if self.tmpLink!=None:
            self.atomspace.remove(self.tmpLink)


class HobbsAgent(MindAgent):

    '''
    Does anaphora resolutions by doing Breadth-First search on the parse tree, rejects any antecedents which are matched by filters
    '''

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

        self.numOfFilters=7

        self.logfile=open('/tmp/results.txt', 'w')

    def bindLinkExe(self,anchorNode, target, command,resultNode,atomType):

        '''
        Just combines all the steps of executing a scheme command into a single function.
        '''

        exe=BindLinkExecution(self.atomspace,anchorNode, target, command,resultNode,atomType)
        exe.execution()
        rv=exe.returnResult()
        exe.clear()
        return rv

    def StringToNumber(self,str):
        return int(str)

    def getWordNumber(self,node):
        return self.wordNumber[node.name]

    def getSentenceNumber(self,node):

        '''
        Given a ParseNode, returns a SentenceNumber of a SentenceNode associated with it.
        '''

        rv=self.bindLinkExe(self.currentTarget,node,'(cog-bind getNumberNode_ParseNode)',self.currentResult,types.NumberNode)
        return int(rv[0].name)

    def sortNodes(self,list,keyFunc):

        '''
        Sorts nodes according to their word sequence number and returns the sorted list.
        '''
        return sorted(list,key=keyFunc)

    def getChildren(self,node):

        '''
        Returns a sorted list of children nodes of current node.
        '''

        rv=self.bindLinkExe(self.currentTarget,node,'(cog-bind getChildren)',self.currentResult,types.WordInstanceNode)
        return self.sortNodes(rv,self.getWordNumber)

    def propose(self,node):
        '''
        It iterates all filters, reject the antecedent or "node" if it's matched by any filters.
        '''

        self.currentResolutionLink_pronoun=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentPronoun, node], TruthValue(1.0, 100))
        rejected = False
        filterNumber=-1
        for index in range(1,self.numOfFilters):
            command='(cog-bind-crisp filter-#'+str(index)+')'
            rv=self.bindLinkExe(self.currentProposal,node,command,self.currentResult,types.AnchorNode)
            if len(rv)>0:
                '''
                Reject it
                '''
                rejected = True
                filterNumber=index
                break

        if not rejected:
            #self.bindLinkExe(self.currentProposal,node,'(cog-bind propose)',None,None)
            #print("accepted "+node.name,file=self.logfile)
            print("accepted "+node.name)
        #else:
            #print("rejected "+node.name+" by filter-#"+str(filterNumber))

    def Checked(self,node):

        '''
        Since graph is not necessarily a forest, this agent actually does a Breadth-First search on a general graph for
        each pronoun, so we need to avoid cycling around the graph by marking each node as checked if we have visited it once.
        '''

        if node.name in self.checked:
            return True
        self.checked[node.name]=True
        return False

    def bfs(self,node):

        '''
        Does a Breadth-First search, starts with "node"
        '''

        if node==None:
            #print("found you bfs")
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
        return self.bindLinkExe(None,None,'(cog-bind getPronouns)',self.unresolvedReferences,types.WordInstanceNode)

    def getRoots(self):

        '''
        Return a list of roots(incoming degree of 0)
        '''

        self.bindLinkExe(None,None,'(cog-bind-crisp connectRootsToParseNodes)',None,None)
        rv= self.bindLinkExe(None,None,'(cog-bind getAllParseNodes)',self.currentResult,types.ParseNode)
        return self.sortNodes(rv,self.getSentenceNumber)

    def getRootOfNode(self,target):
        '''
        Naive approach, but works
        '''
        return self.roots[len(self.roots)-1]

    def  previousRootExist(self,root):

        '''
        "previous" means that a root with smaller word sequence number than the word sequence number of current "roots".
        '''
        return not self.roots[0].name==root.name

    def getPrevious(self,root):

        '''
        Return a previous root.
        '''

        rootNumber=self.getSentenceNumber(root)
        for root in reversed(self.roots):
            number=self.getSentenceNumber(root)
            if number<rootNumber:
                return root
        #print("Impossible")

    def getAllNumberNodes(self):

        '''
        Finds word sequence number for each word
        '''

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

        data=["opencog/nlp/anaphora/rules/getChildren.scm",
              "opencog/nlp/anaphora/rules/getNumberNode_WordInstanceNode.scm",
              "opencog/nlp/anaphora/rules/getNumberNode_ParseNode.scm",
              "opencog/nlp/anaphora/rules/connectRootsToParseNodes.scm",
              "opencog/nlp/anaphora/rules/getPronouns.scm",
              "opencog/nlp/anaphora/rules/propose.scm",
              "opencog/nlp/anaphora/rules/getResults.scm",
              "opencog/nlp/anaphora/rules/getAllNumberNodes.scm",
              "opencog/nlp/anaphora/rules/getAllParseNodes.scm",

              "opencog/nlp/anaphora/rules/filtersGenerator.scm",

              "opencog/nlp/anaphora/rules/filters/filter-#1.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#2.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#3.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#4.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#5.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#6.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#7.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#8.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#9.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#10.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#11.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#12.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#13.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#14.scm",
              ]

        self.numOfFilters=len(data)
        for item in data:
            load_scm(atomspace, item)

        self.getAllNumberNodes()
        self.pronouns = self.getPronouns()
        self.roots = self.getRoots()


    def printResults(self):

        '''
        Currently, this function is not used.
        '''

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

            '''
            Binds current "pronoun" with "currentPronounNode".
            This part is used by pattern matcher.
            '''

            tmpLink=self.atomspace.add_link(types.ListLink, [self.currentPronounNode, pronoun], TruthValue(1.0, 100))
            self.currentPronoun=pronoun
            root=self.getRootOfNode(pronoun)
            print("\nResolving...........")
            print(pronoun)

            while True:
                if root==None:
                    #print("found you while")
                    break
                self.bfs(root)
                if self.previousRootExist(root):
                    root=self.getPrevious(root)
                else:
                    break
            self.atomspace.remove(tmpLink)

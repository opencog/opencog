
from __future__ import print_function
from pprint import pprint
from opencog.cogserver import MindAgent
from opencog.atomspace import types, AtomSpace, TruthValue
from opencog.scheme_wrapper import load_scm,scheme_eval_h,scheme_eval, __init__
from opencog import logger

import Queue
import time

__author__ = 'Hujie Wang'
LOG_LEVEL="fine"
log = logger.create_logger("/tmp/hobbs.log")
log.set_level(LOG_LEVEL)

'''
========================================
Configurations
'''

'''
Number of searching sentences(including the one contains the pronoun)
'''

NUMBER_OF_SEARCHING_SENTENCES = 3

'''
Suppose the decreasing rate is x, then
the ith accepted candidate will have confidence value of
(x^(i-1))(1-x)  i starts at 1.
'''

CONFIDENCE_DECREASING_RATE=0.7

'''
Strength for accepted antecedents
'''

STRENGTH_FOR_ACCEPTED_ANTECEDENTS=0.98

'''
Truth Value for antecendents which have been filtered out by filters.
'''

TV_FOR_FILTERED_OUT_ANTECEDENTS=TruthValue(0.02, TruthValue().confidence_to_count(0.9))

'''
========================================
'''
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

        self.currentPronoun = None
        self.currentPronounNode = None
        self.currentTarget = None
        self.currentResult = None
        self.currentProposal = None
        self.unresolvedReferences=None
        self.pronounNumber = None

        self.pronouns = None
        self.roots = None

        self.confidence = 1.0

        self.numOfFilters=7
        self.number_of_searching_sentences=3
        self.DEBUG = True

        log.fine("\n===========================================================\n Starting hobbs agent.....\n=========================================================== ")

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

        '''
        Converts a string to an integer.
        '''

        return int(str)

    def getWordNumber(self,node):

        '''
        Returns the WordSequence number associated with the 'node'
        '''

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

    def generateReferenceLink(self,anaphora,antecedent,tv):
        '''
        Generates a reference Link for a pair of anaphora and antecedent with confidence "confidence".
        '''

        link = self.atomspace.add_link(types.ReferenceLink, [anaphora, antecedent], tv)
        log.fine("Generated a Reference :\n")
        log.fine("{0}\n".format(link))
        log.fine("===========================================================")

    def getConjunction(self,node):

        '''
        Returning the other part of a conjunction if conjunction exists and anaphor is "Plural"
        '''

        return self.bindLinkExe(self.currentProposal,node,'(cog-bind-crisp getConjunction)',self.currentResult,types.WordInstanceNode)

    def checkConjunctions(self,node):

        '''
        Checking if conjunction resolution applies to the "node", returning True if it applies, False otherwise.
        '''

        conjunction=self.getConjunction(node);

        if len(conjunction)>0:

            conjunction_list=[]
            conjunction_list.append(node)
            conjunction_list.extend(conjunction)

            # We don't want to output this to unit tests
            if self.DEBUG and filter!=-1:
                print("accepted \n"+str(conjunction_list))
            log.fine("accepted \n"+str(conjunction_list))
            self.generateReferenceLink(self.currentPronoun,self.atomspace.add_link(types.AndLink, conjunction_list, TruthValue(1.0, TruthValue().confidence_to_count(1.0))),TruthValue(STRENGTH_FOR_ACCEPTED_ANTECEDENTS, TruthValue().confidence_to_count(self.confidence)))
            self.confidence=self.confidence*CONFIDENCE_DECREASING_RATE
            return True
        return False

    def propose(self,node,filter=-1):
        '''
        It iterates all filters, reject the antecedent or "node" if it's matched by any filters.
        '''

        self.currentResolutionLink_pronoun=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentPronoun, node], TruthValue(1.0, 100))
        rejected = False
        filterNumber=-1

        self.checkConjunctions(node)

        start=1
        end=self.numOfFilters+1

        '''
        For debugging purposes.
        '''
        if filter!=-1:
            start=filter
            end=filter+1

        for index in range(start,end):
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

            # We don't want to output this to unit tests
            if self.DEBUG:
                    print("accepted "+node.name)
            log.fine("accepted "+node.name)
            self.generateReferenceLink(self.currentPronoun,node,TruthValue(STRENGTH_FOR_ACCEPTED_ANTECEDENTS, TruthValue().confidence_to_count(self.confidence)))
            self.confidence=self.confidence*CONFIDENCE_DECREASING_RATE
        else:
            self.generateReferenceLink(self.currentPronoun,node,TV_FOR_FILTERED_OUT_ANTECEDENTS)
            #if self.DEBUG:
                   # print("rejected "+node.name+" by filter-#"+str(index))

        self.atomspace.remove(self.currentResolutionLink_pronoun)
        return not rejected

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

        '''
        rv is used for unit tests
        '''
        rv=[]

        if node==None:
            #print("found you bfs")
            return
        q=Queue.Queue()
        q.put(node)
        while not q.empty():
            front=q.get()
            rv.append(front)
            self.propose(front)
            children=self.getChildren(front)
            if len(children)>0:
                for node in children:
                    if not self.Checked(node):
                        q.put(node)
        return rv

    def getWords(self):

        '''
        Returns a list of words in the atomspace
        '''

        rv=self.bindLinkExe(None,None,'(cog-bind-crisp getWords)',self.currentResult,types.WordInstanceNode)
        return self.sortNodes(rv,self.getWordNumber)

    def getTargets(self,words):

        '''
        Returns a list of references needed to be resolved.
        '''

        targets=[]
        for word in words:
            matched=False
            for index in range(1,self.numOfPrePatterns+1):
                command='(cog-bind-crisp pre-process-#'+str(index)+')'
                rv=self.bindLinkExe(self.currentTarget,word,command,self.currentResult,types.AnchorNode)
                if len(rv)>0:
                    matched=True
                    break
            if matched:
                targets.append(word)
        return targets

    def getPronouns(self):
        rv=self.bindLinkExe(None,None,'(cog-bind-crisp getPronouns)',self.unresolvedReferences,types.WordInstanceNode)
        return self.sortNodes(rv,self.getWordNumber)

    def getRoots(self):

        '''
        Return a list of roots(incoming degree of 0)
        '''

        self.bindLinkExe(None,None,'(cog-bind-crisp connectRootsToParseNodes)',None,None)
        rv= self.bindLinkExe(None,None,'(cog-bind getAllParseNodes)',self.currentResult,types.ParseNode)
        return self.sortNodes(rv,self.getSentenceNumber)

    def getRootOfNode(self,target):
        '''
        Returns a ParseNode associated with the "target"
        '''

        rv=self.bindLinkExe(self.currentTarget,target,'(cog-bind-crisp getParseNode)',self.currentResult,types.ParseNode)
        return rv[0]

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
        '''
        Initializes necessary variables. Loads rules.
        '''

        self.atomspace = atomspace

        self.PleonasticItNode=atomspace.add_node(types.AnchorNode, 'Pleonastic-it', TruthValue(1.0, 100))
        self.currentPronounNode = atomspace.add_node(types.AnchorNode, 'CurrentPronoun', TruthValue(1.0, 100))
        self.currentTarget = atomspace.add_node(types.AnchorNode, 'CurrentTarget', TruthValue(1.0, 100))
        self.currentResult = atomspace.add_node(types.AnchorNode, 'CurrentResult', TruthValue(1.0, 100))
        self.currentProposal = atomspace.add_node(types.AnchorNode, 'CurrentProposal', TruthValue(1.0, 100))
        self.unresolvedReferences=atomspace.add_node(types.AnchorNode, 'Recent Unresolved references', TruthValue(1.0, 100))
        self.resolvedReferences=atomspace.add_node(types.AnchorNode, 'Resolved references', TruthValue(1.0, 100))
        self.currentResolutionNode=atomspace.add_node(types.AnchorNode, 'CurrentResolution', TruthValue(1.0, 100))
        self.currentResolutionLink_proposal=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentProposal], TruthValue(1.0, 100))
        self.currentResolutionLink_pronoun=self.atomspace.add_link(types.ListLink, [self.currentResolutionNode, self.currentPronounNode], TruthValue(1.0, 100))
        self.pronounNumber = -1

        data=["opencog/nlp/anaphora/rules/getChildren.scm",
              "opencog/nlp/anaphora/rules/getNumberNode_WordInstanceNode.scm",
              "opencog/nlp/anaphora/rules/getNumberNode_ParseNode.scm",
              "opencog/nlp/anaphora/rules/connectRootsToParseNodes.scm",
              "opencog/nlp/anaphora/rules/getAllNumberNodes.scm",
              "opencog/nlp/anaphora/rules/getAllParseNodes.scm",
              "opencog/nlp/anaphora/rules/getConjunction.scm",
              "opencog/nlp/anaphora/rules/getParseNode.scm",
              "opencog/nlp/anaphora/rules/getWords.scm",
              "opencog/nlp/anaphora/rules/isIt.scm",

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
              "opencog/nlp/anaphora/rules/filters/filter-#15.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#16.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#17.scm",
              "opencog/nlp/anaphora/rules/filters/filter-#18.scm",

              "opencog/nlp/anaphora/rules/pre-process/pre-process-#1.scm",
              "opencog/nlp/anaphora/rules/pre-process/pre-process-#2.scm",
              "opencog/nlp/anaphora/rules/pre-process/pre-process-#3.scm",

              "opencog/nlp/anaphora/rules/pleonastic-it/pleonastic-it-#1.scm",
              "opencog/nlp/anaphora/rules/pleonastic-it/pleonastic-it-#2.scm",
              "opencog/nlp/anaphora/rules/pleonastic-it/pleonastic-it-#3.scm",

              ]

        self.numOfFilters=20
        self.numOfPrePatterns=3
        self.numOfPleonasticItPatterns=3

        for item in data:
            load_scm(atomspace, item)

        self.getAllNumberNodes()
        self.pronouns=self.getTargets(self.getWords())
        self.roots = self.getRoots()


    def addPronounToResolvedList(self,node):
        '''
        Mark current pronoun as resolved.
        '''

        self.atomspace.add_link(types.ListLink,[self.resolvedReferences,node],TruthValue(1.0, 100))

    def pleonastic_it(self,node):
        '''
        Check if the node is the word "it".
        '''
        matched=False
        rv=self.bindLinkExe(self.currentTarget,node,'(cog-bind-crisp isIt)',self.currentResult,types.AnchorNode)
        if len(rv)>0:
            for index in range(1,self.numOfPleonasticItPatterns+1):
                command='(cog-bind-crisp pleonastic-it-#'+str(index)+')'
                rv=self.bindLinkExe(self.currentTarget,node,command,self.currentResult,types.AnchorNode)
                if len(rv)>0:
                    matched=True
                    break
                #print("rejected "+node.name+" by filter-#"+str(index))

        return matched

    def run(self, atomspace):
        self.initilization(atomspace)

        for pronoun in self.pronouns:


            self.checked.clear()
            self.pronounNumber=self.getWordNumber(pronoun)
            self.confidence=1-CONFIDENCE_DECREASING_RATE

            '''
            Binds current "pronoun" with "currentPronounNode".
            This part is used by pattern matcher.
            '''

            tmpLink=self.atomspace.add_link(types.ListLink, [self.currentPronounNode, pronoun], TruthValue(1.0, 100))
            self.currentPronoun=pronoun
            root=self.getRootOfNode(pronoun)

            if self.DEBUG:
                print("Resolving....")
                print(pronoun)
            log.fine("Resolving \n{0}".format(pronoun))

            '''
            Check if it's a pleonastic it.
            '''

            if self.pleonastic_it(pronoun):
                self.generateReferenceLink(pronoun,self.PleonasticItNode,TruthValue(STRENGTH_FOR_ACCEPTED_ANTECEDENTS, TruthValue().confidence_to_count(self.confidence)))
                self.confidence=self.confidence*CONFIDENCE_DECREASING_RATE
                if self.DEBUG:
                    print("accepted "+self.PleonasticItNode.name)
                    log.fine("accepted "+self.PleonasticItNode.name)

            sent_counter=1;
            while True:
                if root==None:
                    break
                self.bfs(root)
                if self.previousRootExist(root) and sent_counter<=NUMBER_OF_SEARCHING_SENTENCES:
                    root=self.getPrevious(root)
                    sent_counter=sent_counter+1
                else:
                    break
            self.atomspace.remove(tmpLink)
            self.addPronounToResolvedList(pronoun)

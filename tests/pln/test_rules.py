__author__ = 'root'

import unittest
raise unittest.SkipTest("Unit test disabled because Python PLN is deprecated.")

from opencog.atomspace import AtomSpace, TruthValue, Atom, Handle
from opencog.atomspace import types, is_a, get_type, get_type_name
from opencog.scheme_wrapper import load_scm, scheme_eval, scheme_eval_h, __init__
from pln.chainers import Chainer
from pln.rules import *
from unittest import TestCase
import re
import os

__VERBOSE__ = False

# Set to True to search for needed .scm files in default IN-SOURCE build location, e.g. to write unit tests in the IDE
# Set to False to search for needed .scm files based on environment variables PROJECT_SOURCE_DIR and PROJECT_BINARY_DIR
__DEV_MODE__ = False


# Get the current directory for this test so this test works no matter
# which directory is the current directory when nosetests is run. The
# other paths will be relative to the directory for this.
test_pln_directory = os.path.abspath(os.path.dirname(__file__))


class PLNUnitTester(TestCase):
    def setUp(self):
        self.atomSpaceFileData = AtomSpace()
        self.atomSpaceInputs = AtomSpace()
        self.atomSpaceExpected = AtomSpace()
        self.testFiles = []

        self.chainer = None

        # Works:
        self.addTestFile("AndRule_new.scm")
        self.addTestFile("BooleanTransformationRule_new.scm")
        self.addTestFile("DeductionRule_InheritanceLink.scm")
        self.addTestFile("InheritanceRule.scm")

        # Unit test disabled, see: https://github.com/opencog/opencog/issues/1280
        # self.addTestFile("InductionRule_InheritanceLink.scm")

        self.addTestFile("AbductionRule_InheritanceLink.scm") # Under investigation
        self.addTestFile("InversionRule_InheritanceLink.scm")
        self.addTestFile("OrCreationRule.scm")
        self.addTestFile("OrRule_new.scm")
        self.addTestFile("NotCreationRule.scm")
        self.addTestFile("TransitiveSimilarityRule_SimilarityLink.scm")
        self.addTestFile("AndAs1stArgInsideLinkRule_InheritanceLink.scm")
        self.addTestFile("AndAs2ndArgInsideLinkRule_InheritanceLink.scm")
        self.addTestFile("SatisfyingSetToConceptRule.scm")
        # context rules
        self.addTestFile("InheritanceToContextRule.scm")
        self.addTestFile("ContextToInheritanceRule.scm")
        self.addTestFile("ContextToEvaluationRule.scm")
        self.addTestFile("ContextToSubsetRule.scm")
        self.addTestFile("EvaluationToContextRule.scm")
        self.addTestFile("SubsetToContextRule.scm")

        # Testing (just a placeholder for where to put tests while...testing them)
        #self.addTestFile("SimilarityRule_And.scm")

        # Amen is looking at this one:
        #self.addTestFile("GeneralEvaluationToMemberRule.scm")

        # Following tests give results that don't match or exceed expectations:
        #self.addTestFile("SubsetAS_new.scm") # Unit test has been failing
        #self.addTestFile("LionTigerAS_new.scm") # Lots of stuff
        #self.addTestFile("IntensionalLinkEvaluationRule_new.scm") # Creates an IntensionalSimilarityLink though no such rule was supplied.

        # Following tests don't give chaining results, since we're testing forward chaining only right now lol.
        #self.addTestFile("AndBulkEvaluationRule_new.scm")
        #self.addTestFile("AndBulkEvaluationRule3EvaluationLinks_new.scm")
        #self.addTestFile("AndBulkEvaluationRule3_new.scm")
        #self.addTestFile("AndBulkEvaluationRuleEvaluationLinks_new.scm")
        #self.addTestFile("AndBulkEvaluationRulePredicates_new.scm")
        #self.addTestFile("NegatedAndBulkEvaluationRule3EvaluationLinks_new.scm")

        # Doesn't work, as the unit test setup doesn't allow for changing TV's (YET)
        # self.addTestFile("AndBreakdownRule.scm")

    def tearDown(self):
        del self.atomSpaceFileData
        del self.atomSpaceInputs
        del self.atomSpaceExpected
        del self.chainer

    def test_all(self):
        if len(self.testFiles) == 0:
            print "No test files have been selected."
        else:
            for testFile in self.testFiles:
                print "Testing file: " + testFile
                self.run_file(testFile)

    def addTestFile(self, testFile):
        # Again, default to dev mode
        ruleFolder = str(os.getcwd()) + "/new_scm_tests/"

        if not __DEV_MODE__:
            ruleFolder = test_pln_directory + "/new_scm_tests/"

        fullTestFile = ruleFolder + testFile

        self.testFiles.append(fullTestFile)

    def run_chaining_steps(self):
        if __VERBOSE__:
            print "Rules that can be applied:"
            print [" *" + r.name for r in self.chainer.rules]

        numberOfSteps = 1
        try:
            numberOfStepsNode = self.get_predicate_arguments(self.atomSpaceFileData, "forwardSteps")
            numberOfSteps = int(numberOfStepsNode[0].name)
        except:
            numberOfSteps = 1

        while numberOfSteps > 0:
            result = self.chainer.forward_step()

            if not result is None:
                numberOfSteps -= 1
                if __VERBOSE__:
                    print result

    def reset_atom_spaces(self):
        self.reset_atom_space(self.atomSpaceFileData)
        self.reset_atom_space(self.atomSpaceInputs)
        self.reset_atom_space(self.atomSpaceExpected)

    def load_file_into_atomspace(self, fullPath, atomSpace):
        load_scm(atomSpace, fullPath)

    def reset_atom_space(self, atomspaceToReset):
        atomspaceToReset.clear()

        # Default to dev mode
        coreTypes = "opencog/scm/core_types.scm"
        utilities = "opencog/scm/utilities.scm"
        plnTypes = "opencog/reasoning/pln/pln_types.scm"

        if not __DEV_MODE__:
            coreTypes = os.path.join(test_pln_directory, 
                    'opencog/scm/core_types.scm')
            utilities = os.path.join(test_pln_directory, 
                    'opencog/scm/utilities.scm')
            plnTypes = os.path.join(test_pln_directory, 
                    '../../../build/opencog/reasoning/pln/pln_types.scm')

            # Guile can't handle paths with relative '../..' etc., so
            # normalize to a full path for guile.
            coreTypes = os.path.normpath(coreTypes)
            utilities = os.path.normpath(utilities)
            plnTypes = os.path.normpath(plnTypes)

        self.load_file_into_atomspace(coreTypes, atomspaceToReset)
        self.load_file_into_atomspace(utilities, atomspaceToReset)
        self.load_file_into_atomspace(plnTypes, atomspaceToReset)

    def fill_atomspace(self, atomspace, atomList):
        for atom in atomList:
            self.transfer_atom(atomspace, atom)

    def load_inputs(self):
        self.fill_atomspace(self.atomSpaceInputs, self.get_predicate_arguments(self.atomSpaceFileData, "inputs"))

        if __VERBOSE__:
            self.print_atomspace(self.atomSpaceInputs)

    def print_atomspace(self, atomspace):
        allNodes = atomspace.get_atoms_by_type(types.Node)

        for node in allNodes:
            print str(node.type) + " called " + node.name

        allLinks = atomspace.get_atoms_by_type(types.Link)

        for link in allLinks:
            print str(link.type) + " called " + link.name
            for out in link.out:
                print "  has a node of type " + str(out.type) + " called " + out.name

    def load_outputs(self):
        self.fill_atomspace(self.atomSpaceExpected, self.get_predicate_arguments(self.atomSpaceFileData, "outputs"))

    def load_rules(self):
        rules = self.get_predicate_arguments(self.atomSpaceFileData, "rules")

        allRules = AllRules(self.atomSpaceInputs, self.chainer)

        for rule in rules:
            #allRules.import_rule(rule, self.chainer)
            self.chainer.add_rule(allRules.lookup_rule(rule))

    def atomspace_links_to_list(self, atomSpace):
        result = []

        expectedLinks = atomSpace.get_atoms_by_type(types.Link)

        for link in expectedLinks:
            atomStringToCheckFor = atomSpace.get_atom_string(link.h)
            if not "standardize_apart" in atomStringToCheckFor:
                atomStringToCheckFor = re.sub(r"\[[0-9]+\]", "", atomStringToCheckFor)
                result.append(atomStringToCheckFor)

        return result

    def check_atomspace_contains_atomspace(self, atomSpaceChecklist, atomSpaceToCheck):
        result = True
        error = ""

        expectedList = self.atomspace_links_to_list(atomSpaceChecklist)
        actualList = self.atomspace_links_to_list(atomSpaceToCheck)

        for listItem in expectedList:
            if not (listItem in actualList):
                if not "VariableNode" in listItem:
                    if not "ListLink" in listItem:
                        result = False
                        error = "Unable to find \n" + listItem
                        print error

        return result

    def verify_result(self):
        allPredictedItemsExist = False
        allItemsWerePredicted = False

        print "Checking if all predicted items were created:"

        if self.check_atomspace_contains_atomspace(self.atomSpaceExpected, self.atomSpaceInputs):
            print "  Yes, all predicted items exist"
            allPredictedItemsExist = True
        else:
            print "  No, not all predicted items were created (see above for details)"
        print "Checking if all created items were predicted:"

        if self.check_atomspace_contains_atomspace(self.atomSpaceInputs, self.atomSpaceExpected):
            print "  Yes, all created items were predicted"
            allItemsWerePredicted = True
        else:
            print "  No, some created items were not predicted (see above for details)"

        self.assertTrue(allPredictedItemsExist and allItemsWerePredicted)

    def run_file(self, filename):
        if (os.path.exists(filename)):
            self.reset_atom_spaces()

            self.chainer = Chainer(self.atomSpaceInputs,
                                   stimulateAtoms=False, agent=self,
                                   learnRuleFrequencies=False,
                                   allow_output_with_variables=True,
                                   allow_backchaining_with_variables=True)


            self.load_file_into_atomspace(filename, self.atomSpaceFileData)

            self.load_inputs()
            self.load_outputs()
            self.load_rules()

            self.run_chaining_steps()

            self.verify_result()
        else:
            print "File does not exist: " + filename

    def transfer_atom(self, new_atomspace, atom):
        """
        transfer (or rather copy) an atom from one atomspace to
        another. Assumes that both AtomSpaces have the same list of
        Atom types!
        returns the equivalent of atom in new_atomspace. creates it if
        necessary, including the outgoing set of links.
        """
        # The AtomSpace probably clones the TV objects, and it wouldn't
        # matter much anyway
        #tv = TruthValue(atom.tv.mean, atom.tv.count)

        if atom.is_node():
            return new_atomspace.add_node(atom.type, atom.name, tv=atom.tv)
        else:
            outgoing = [self.transfer_atom(new_atomspace, out)
                        for out in atom.out]
            return new_atomspace.add_link(atom.type, outgoing, tv=atom.tv)

    def get_predicate_arguments(self, atomspace, predicate_name):
        """
        Find the EvaluationLink for the predicate, and return the list
        of arguments (as a python list of Atoms). There must be only
        one EvaluationLink for it
        """
        var = self.new_variable(atomspace)
        template = self.link(atomspace, types.EvaluationLink,
                             [self.node(atomspace, types.PredicateNode, predicate_name),
                              var])

        queries = self.lookup_atoms(atomspace, template)
        # It will often find the original template in the results!
        queries.remove(template)
        #queries = [query for query in queries if query.tv.count > 0]
        if len(queries) != 1:
            raise ValueError("Predicate " + predicate_name +
                             " must have 1 EvaluationLink")
        return queries[0].out[1].out

    def lookup_atoms(self, atomspace, template):
        if len(self.variables(template)) == 0:
            return [template]

        if template.type == types.VariableNode:
            root_type = types.Atom
            atoms = atomspace.get_atoms_by_type(root_type)
        else:
            # If the atom is a link with all variables below it, then
            # lookup all links of that type. If it has any nodes
            # (which aren't VariableNodes!), then lookup the incoming
            # set for that node
            first_node = self.get_first_node(template)
            if first_node is None:
                root_type = template.type
                atoms = atomspace.get_atoms_by_type(root_type)
            else:
                atoms = self.get_incoming_recursive(first_node)

        return atoms

    def new_variable(self, atomspace, prefix='$pln_var_'):
        return atomspace.add_node(types.VariableNode,
                                        prefix,
                                        prefixed=True)

    def link(self, atomspace, type, out):
        return atomspace.add_link(type, out)

    def node(self, atomspace, type, name):
        return atomspace.add_node(type, name)

    def variables(self, atom):
        """
        Find all the variables in an expression (which may be
        repeated)
        """
        if atom.is_node():
            if self.is_variable(atom):
                return [atom]
            else:
                return []
        else:
            result = []
            for o in atom.out:
                result += self.variables(o)
            return result

    def get_first_node(self, atom):
        """
        Using a depth first search on the link, return the first Node
        found. If atom is a Node just return that.
        """
        if atom.is_node() and not self.is_variable(atom):
            return atom
        else:
            for o in atom.out:
                ret = self.get_first_node(o)
                if not ret is None:
                    return ret
            return None

    def get_incoming_recursive(self, atom):
        inc = atom.incoming
        ret = []
        ret += inc
        for link in inc:
            ret += self.get_incoming_recursive(link)
        return ret

    def is_variable(self, atom):
        return atom.is_a(types.VariableNode)

class AllRules(object):
    def __init__(self, atomspace, chainer):
        # contains every rule
        self.chainer = Chainer(atomspace,
                               stimulateAtoms=False,
                               agent=self,
                               learnRuleFrequencies=False)

        # contains only some rules
        self.test_chainer = chainer

        conditional_probability_types = [types.InheritanceLink,
                                         types.ImplicationLink,
                                         types.PredictiveImplicationLink]
        similarity_types = [types.SimilarityLink, types.EquivalenceLink]

        for link_type in conditional_probability_types:
            self.chainer.add_rule(InversionRule(self.chainer, link_type)) # Rule test exists.
            self.chainer.add_rule(DeductionRule(self.chainer, link_type)) # Rule test exists.
            self.chainer.add_rule(InductionRule(self.chainer, link_type)) # Rule test exists.
            self.chainer.add_rule(AbductionRule(self.chainer, link_type)) # Rule test exists.
            # Seems better than Modus Ponens - it doesn't make anything up
            self.chainer.add_rule(TermProbabilityRule(self.chainer, link_type))
            self.chainer.add_rule(ModusPonensRule(self.chainer, link_type))
            self.chainer.add_rule(PreciseModusPonensRule(self.chainer, link_type))
            self.chainer.add_rule(AndAs1stArgInsideLinkRule(self.chainer, link_type))
            self.chainer.add_rule(AndAs2ndArgInsideLinkRule(self.chainer, link_type))

        for link_type in similarity_types:
            # SimilarityLinks don't require an InversionRule obviously
            self.chainer.add_rule(
                TransitiveSimilarityRule(self.chainer, link_type)) # Rule test exists.
            self.chainer.add_rule(
                SymmetricModusPonensRule(self.chainer, link_type))

        self.chainer.add_rule(EvaluationImplicationRule(self.chainer)) # No test, appears to produce a TV not new nodes.

        # These two Rules create mixed links out of intensional and
        # extensional links
        self.chainer.add_rule(InheritanceRule(self.chainer)) # Rule test exists.
        self.chainer.add_rule(SimilarityRule(self.chainer))

        for link_type in conditional_probability_types:
            self.chainer.add_rule(AndToSubsetRule1(self.chainer, link_type))

            for N in xrange(2, 8):
                self.chainer.add_rule(AndToSubsetRuleN(self.chainer, link_type, N))

        # boolean links
        for rule in create_and_or_rules(self.chainer, 2, 8):
            self.chainer.add_rule(rule)
        for N in xrange(2, 8):
            self.chainer.add_rule(
                boolean_rules.AndBulkEvaluationRule(self.chainer, N))
        for N in xrange(3, 8):
            self.chainer.add_rule(
                boolean_rules.NegatedAndBulkEvaluationRule(self.chainer, N))

        general_evaluation_to_member_rules = create_general_evaluation_to_member_rules(self.chainer)
        for rule in general_evaluation_to_member_rules:
            self.chainer.add_rule(rule)

        # create probabilistic logical links out of MemberLinks

        self.chainer.add_rule(AndEvaluationRule(self.chainer))
        self.chainer.add_rule(OrEvaluationRule(self.chainer))

        self.chainer.add_rule(ExtensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(IntensionalLinkEvaluationRule(self.chainer))
        self.chainer.add_rule(SubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(NegatedSubsetEvaluationRule(self.chainer))
        self.chainer.add_rule(
            ExtensionalSimilarityEvaluationRule(self.chainer))
        self.chainer.add_rule(
            IntensionalInheritanceEvaluationRule(self.chainer))
        self.chainer.add_rule(
            IntensionalSimilarityEvaluationRule(self.chainer))
        self.chainer.add_rule(SatisfyingSetToConceptRule(self.chainer, 1))

        # context rules
        self.chainer.add_rule(InheritanceToContextRule(self.chainer))
        self.chainer.add_rule(EvaluationToContextRule(self.chainer))
        self.chainer.add_rule(SubsetToContextRule(self.chainer))
        self.chainer.add_rule(ContextToInheritanceRule(self.chainer))
        self.chainer.add_rule(ContextToEvaluationRule(self.chainer))
        self.chainer.add_rule(ContextToSubsetRule(self.chainer))

        # self.member_rules = [GeneralEvaluationToMemberRule(self.chainer),
        #     MemberToEvaluationRule(self.chainer)]
        # self.member_rules += \
        #     create_general_evaluation_to_member_rules(self.chainer)
        # for rule in self.member_rules:
        #     self.chainer.add_rule(rule)

        # It's important to have both of these
        self.chainer.add_rule(MemberToInheritanceRule(self.chainer))
#        self.chainer.add_rule(InheritanceToMemberRule(self.chainer))

        # AttractionLink could be useful for causality
        self.chainer.add_rule(AttractionRule(self.chainer))

        self.chainer.add_rule(ScholemRule(self.chainer))

        for rule in create_temporal_rules(self.chainer):
            self.chainer.add_rule(rule)

        boolean_transformation_rules = create_boolean_transformation_rules(self.chainer)
        for rule in boolean_transformation_rules:
            self.chainer.add_rule(rule)

    def lookup_rule(self, rule_schema_node):
        rule = self.chainer.lookup_rule(rule_schema_node.name)
        rule._chainer = self.test_chainer
        return rule

def main():
    pln_unit_tester = PLNUnitTester()
    pln_unit_tester.test_all()


if __name__ == "__main__":
    main()


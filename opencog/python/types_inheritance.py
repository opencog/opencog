##
# @file types_inheritance.py
# @brief 
# @author Dingjie.Wang
# @version 1.0
# @date 2012-08-04
import networkx as nx
from opencog.atomspace import types as t
from m_util import log
# graph of types infomation
types_graph = nx.DiGraph()
name_type_dict = { }
type_name_dict = { }
# parent -> child
types_graph.add_edge('Link','UnorderedLink')
types_graph.add_edge('Link','OrderedLink')
types_graph.add_edge('Link','HebbianLink')
types_graph.add_edge('FeatureLink','PartOfSpeechLink')
types_graph.add_edge('ConceptNode','WordInstanceNode')
types_graph.add_edge('ConceptNode','DefinedFrameNode')
types_graph.add_edge('ConceptNode','WordSenseNode')
types_graph.add_edge('ConceptNode','LemmaNode')
types_graph.add_edge('ConceptNode','ParseNode')
types_graph.add_edge('ConceptNode','DocumentNode')
types_graph.add_edge('ConceptNode','SentenceNode')
types_graph.add_edge('ConceptNode','SemeNode')
types_graph.add_edge('ConceptNode','FeatureNode')
types_graph.add_edge('PredicateNode','LinkGrammarDisjunctNode')
types_graph.add_edge('PredicateNode','DefinedLinguisticRelationshipNode')
types_graph.add_edge('PredicateNode','DefinedFrameElementNode')
types_graph.add_edge('PredicateNode','LinkGrammarRelationshipNode')
types_graph.add_edge('PredicateNode','GroundedPredicateNode')
types_graph.add_edge('PredicateNode','PrepositionalRelationshipNode')
types_graph.add_edge('BindLink','ExistsLink')
types_graph.add_edge('BindLink','ForAllLink')
types_graph.add_edge('BindLink','AverageLink')
types_graph.add_edge('SchemaNode','GroundedSchemaNode')
types_graph.add_edge('SchemaNode','PredicateNode')
types_graph.add_edge('ProcedureNode','SchemaNode')
types_graph.add_edge('ProcedureNode','GroundedProcedureNode')
types_graph.add_edge('SemeNode','SemanticRelationNode')
types_graph.add_edge('OrderedLink','ExecutionOutputLink')
types_graph.add_edge('OrderedLink','BindLink')
types_graph.add_edge('OrderedLink','SubsetLink')
types_graph.add_edge('OrderedLink','SequentialAndLink')
types_graph.add_edge('OrderedLink','IsAcceptableSecondArgLink')
types_graph.add_edge('OrderedLink','HypotheticalLink')
types_graph.add_edge('OrderedLink','ExtensionalSimilarityLink')
types_graph.add_edge('OrderedLink','TailPredictiveImplicationLink')
types_graph.add_edge('OrderedLink','SimultaneousAndLink')
types_graph.add_edge('OrderedLink','AssociativeLink')
types_graph.add_edge('OrderedLink','FeatureLink')
types_graph.add_edge('OrderedLink','AsymmetricHebbianLink')
types_graph.add_edge('OrderedLink','ContextLink')
types_graph.add_edge('OrderedLink','MemberLink')
types_graph.add_edge('OrderedLink','LemmaLink')
types_graph.add_edge('OrderedLink','EvaluationLink')
types_graph.add_edge('OrderedLink','InverseHebbianLink')
types_graph.add_edge('OrderedLink','ListLink')
types_graph.add_edge('OrderedLink','ExecutionLink')
types_graph.add_edge('OrderedLink','IntensionalSimilarityLink')
types_graph.add_edge('OrderedLink','FrameElementLink')
types_graph.add_edge('OrderedLink','IntensionalInheritanceLink')
types_graph.add_edge('OrderedLink','FalseLink')
types_graph.add_edge('OrderedLink','InheritanceLink')
types_graph.add_edge('OrderedLink','PredictiveImplicationLink')
types_graph.add_edge('OrderedLink','MixedImplicationLink')
types_graph.add_edge('OrderedLink','SatisfyingSetLink')
types_graph.add_edge('OrderedLink','AtTimeLink')
types_graph.add_edge('OrderedLink','ExtensionalImplicationLink')
types_graph.add_edge('OrderedLink','EventualSequentialImplication')
types_graph.add_edge('OrderedLink','TrueLink')
types_graph.add_edge('OrderedLink','ImplicationLink')
types_graph.add_edge('OrderedLink','TypedVariableLink')
types_graph.add_edge('OrderedLink','LatestLink')
types_graph.add_edge('OrderedLink','ScholemLink')
types_graph.add_edge('OrderedLink','EventualSequentialAND')
types_graph.add_edge('OrderedLink','CountLink')
types_graph.add_edge('ObjectNode','AccessoryNode')
types_graph.add_edge('ObjectNode','StructureNode')
types_graph.add_edge('ObjectNode','AvatarNode')
types_graph.add_edge('ObjectNode','UnknownObjectNode')
types_graph.add_edge('ObjectNode','HumanoidNode')
types_graph.add_edge('ObjectNode','PetNode')
types_graph.add_edge('GroundedProcedureNode','GroundedSchemaNode')
types_graph.add_edge('GroundedProcedureNode','GroundedPredicateNode')
types_graph.add_edge('HebbianLink','SymmetricHebbianLink')
types_graph.add_edge('HebbianLink','InverseHebbianLink')
types_graph.add_edge('HebbianLink','AsymmetricHebbianLink')
types_graph.add_edge('HebbianLink','SymmetricInverseHebbianLink')
types_graph.add_edge('InheritanceLink','SchemaExecutionLink')
types_graph.add_edge('InheritanceLink','HolonymLink')
types_graph.add_edge('SchemaExecutionLink','SchemaEvaluationLink')
types_graph.add_edge('Node','AGISIMObjectPerceptNode')
types_graph.add_edge('Node','AGISIMTasteNode')
types_graph.add_edge('Node','AGISIMSelfNode')
types_graph.add_edge('Node','ConceptNode')
types_graph.add_edge('Node','AGISIMSmellNode')
types_graph.add_edge('Node','AnchorNode')
types_graph.add_edge('Node','VariableNode')
types_graph.add_edge('Node','AGISIMPixelPerceptNode')
types_graph.add_edge('Node','VariableTypeNode')
types_graph.add_edge('Node','AGIMSIMVisualPerceptNode')
types_graph.add_edge('Node','ProcedureNode')
types_graph.add_edge('Node','AGISIMSoundNode')
types_graph.add_edge('Node','TimeNode')
types_graph.add_edge('Node','ObjectNode')
types_graph.add_edge('Node','NumberNode')
types_graph.add_edge('Node','AGISIMPolygonPerceptNode')
types_graph.add_edge('Node','CWColorNode')
types_graph.add_edge('Node','AGISIMPerceptNode')
types_graph.add_edge('Node','WordNode')
types_graph.add_edge('Node','CWPixelPerceptNode')
types_graph.add_edge('AssociativeLink','WordSenseLink')
types_graph.add_edge('AssociativeLink','ParseLink')
types_graph.add_edge('AssociativeLink','WordInstanceLink')
types_graph.add_edge('AssociativeLink','ReferenceLink')
types_graph.add_edge('UnorderedLink','SymmetricHebbianLink')
types_graph.add_edge('UnorderedLink','SetLink')
types_graph.add_edge('UnorderedLink','SimilarityLink')
types_graph.add_edge('UnorderedLink','SimultaneousEquivalenceLink')
types_graph.add_edge('UnorderedLink','OrLink')
types_graph.add_edge('UnorderedLink','AndLink')
types_graph.add_edge('UnorderedLink','CosenseLink')
types_graph.add_edge('UnorderedLink','EquivalenceLink')
types_graph.add_edge('UnorderedLink','ExtensionalEquivalenceLink')
types_graph.add_edge('UnorderedLink','NotLink')
types_graph.add_edge('UnorderedLink','SymmetricInverseHebbianLink')
types_graph.add_edge('Atom','Node')
types_graph.add_edge('Atom','Link')
types_graph.add_edge('Atom','Atom')
types_graph.add_edge('FeatureNode','PartOfSpeechNode')
types_graph.add_edge('FeatureNode','DefinedLinguisticConceptNode')
types_graph.remove_edge('Atom','Atom')
name_type_dict["EventualSequentialImplication"] = t.EventualSequentialImplication
name_type_dict["FalseLink"] = t.FalseLink
name_type_dict["DefinedFrameNode"] = t.DefinedFrameNode
name_type_dict["InverseHebbianLink"] = t.InverseHebbianLink
name_type_dict["GroundedSchemaNode"] = t.GroundedSchemaNode
name_type_dict["Link"] = t.Link
name_type_dict["EvaluationLink"] = t.EvaluationLink
name_type_dict["TrueLink"] = t.TrueLink
name_type_dict["NumberNode"] = t.NumberNode
name_type_dict["HolonymLink"] = t.HolonymLink
name_type_dict["HebbianLink"] = t.HebbianLink
name_type_dict["TailPredictiveImplicationLink"] = t.TailPredictiveImplicationLink
name_type_dict["SequentialAndLink"] = t.SequentialAndLink
name_type_dict["EventualSequentialAND"] = t.EventualSequentialAND
name_type_dict["SemanticRelationNode"] = t.SemanticRelationNode
name_type_dict["AGISIMObjectPerceptNode"] = t.AGISIMObjectPerceptNode
name_type_dict["StructureNode"] = t.StructureNode
name_type_dict["CWPixelPerceptNode"] = t.CWPixelPerceptNode
name_type_dict["Node"] = t.Node
name_type_dict["ObjectNode"] = t.ObjectNode
name_type_dict["LatestLink"] = t.LatestLink
name_type_dict["SetLink"] = t.SetLink
name_type_dict["PetNode"] = t.PetNode
name_type_dict["AsymmetricHebbianLink"] = t.AsymmetricHebbianLink
name_type_dict["SubsetLink"] = t.SubsetLink
name_type_dict["ReferenceLink"] = t.ReferenceLink
name_type_dict["SimultaneousEquivalenceLink"] = t.SimultaneousEquivalenceLink
name_type_dict["DocumentNode"] = t.DocumentNode
name_type_dict["WordInstanceLink"] = t.WordInstanceLink
name_type_dict["SymmetricHebbianLink"] = t.SymmetricHebbianLink
name_type_dict["VariableTypeNode"] = t.VariableTypeNode
name_type_dict["ParseLink"] = t.ParseLink
name_type_dict["IsAcceptableSecondArgLink"] = t.IsAcceptableSecondArgLink
name_type_dict["SentenceNode"] = t.SentenceNode
name_type_dict["PartOfSpeechNode"] = t.PartOfSpeechNode
name_type_dict["LemmaNode"] = t.LemmaNode
name_type_dict["MixedImplicationLink"] = t.MixedImplicationLink
name_type_dict["ScholemLink"] = t.ScholemLink
name_type_dict["VariableNode"] = t.VariableNode
name_type_dict["AGIMSIMVisualPerceptNode"] = t.AGIMSIMVisualPerceptNode
name_type_dict["NotLink"] = t.NotLink
name_type_dict["CWColorNode"] = t.CWColorNode
name_type_dict["ForAllLink"] = t.ForAllLink
name_type_dict["AGISIMSmellNode"] = t.AGISIMSmellNode
name_type_dict["UnknownObjectNode"] = t.UnknownObjectNode
name_type_dict["DefinedLinguisticConceptNode"] = t.DefinedLinguisticConceptNode
name_type_dict["PredictiveImplicationLink"] = t.PredictiveImplicationLink
name_type_dict["SatisfyingSetLink"] = t.SatisfyingSetLink
name_type_dict["ExtensionalSimilarityLink"] = t.ExtensionalSimilarityLink
name_type_dict["OrLink"] = t.OrLink
name_type_dict["ConceptNode"] = t.ConceptNode
name_type_dict["FrameElementLink"] = t.FrameElementLink
name_type_dict["ExtensionalImplicationLink"] = t.ExtensionalImplicationLink
name_type_dict["AGISIMSoundNode"] = t.AGISIMSoundNode
name_type_dict["AndLink"] = t.AndLink
name_type_dict["OrderedLink"] = t.OrderedLink
name_type_dict["SymmetricInverseHebbianLink"] = t.SymmetricInverseHebbianLink
name_type_dict["GroundedPredicateNode"] = t.GroundedPredicateNode
name_type_dict["PredicateNode"] = t.PredicateNode
name_type_dict["WordSenseLink"] = t.WordSenseLink
name_type_dict["MemberLink"] = t.MemberLink
name_type_dict["AGISIMPixelPerceptNode"] = t.AGISIMPixelPerceptNode
name_type_dict["Atom"] = t.Atom
name_type_dict["IntensionalSimilarityLink"] = t.IntensionalSimilarityLink
name_type_dict["PrepositionalRelationshipNode"] = t.PrepositionalRelationshipNode
name_type_dict["ListLink"] = t.ListLink
name_type_dict["SchemaExecutionLink"] = t.SchemaExecutionLink
name_type_dict["AGISIMPolygonPerceptNode"] = t.AGISIMPolygonPerceptNode
name_type_dict["ExecutionOutputLink"] = t.ExecutionOutputLink
name_type_dict["WordInstanceNode"] = t.WordInstanceNode
name_type_dict["ExtensionalEquivalenceLink"] = t.ExtensionalEquivalenceLink
name_type_dict["AccessoryNode"] = t.AccessoryNode
name_type_dict["AverageLink"] = t.AverageLink
name_type_dict["AtTimeLink"] = t.AtTimeLink
name_type_dict["SimilarityLink"] = t.SimilarityLink
name_type_dict["GroundedProcedureNode"] = t.GroundedProcedureNode
name_type_dict["CountLink"] = t.CountLink
name_type_dict["CosenseLink"] = t.CosenseLink
name_type_dict["ContextLink"] = t.ContextLink
name_type_dict["LinkGrammarDisjunctNode"] = t.LinkGrammarDisjunctNode
name_type_dict["ExecutionLink"] = t.ExecutionLink
name_type_dict["HumanoidNode"] = t.HumanoidNode
name_type_dict["EquivalenceLink"] = t.EquivalenceLink
name_type_dict["AssociativeLink"] = t.AssociativeLink
name_type_dict["AvatarNode"] = t.AvatarNode
name_type_dict["LemmaLink"] = t.LemmaLink
name_type_dict["TypedVariableLink"] = t.TypedVariableLink
name_type_dict["ImplicationLink"] = t.ImplicationLink
name_type_dict["AGISIMTasteNode"] = t.AGISIMTasteNode
name_type_dict["SimultaneousAndLink"] = t.SimultaneousAndLink
name_type_dict["ExistsLink"] = t.ExistsLink
name_type_dict["HypotheticalLink"] = t.HypotheticalLink
name_type_dict["InheritanceLink"] = t.InheritanceLink
name_type_dict["PartOfSpeechLink"] = t.PartOfSpeechLink
name_type_dict["DefinedFrameElementNode"] = t.DefinedFrameElementNode
name_type_dict["FeatureLink"] = t.FeatureLink
name_type_dict["ParseNode"] = t.ParseNode
name_type_dict["SemeNode"] = t.SemeNode
name_type_dict["DefinedLinguisticRelationshipNode"] = t.DefinedLinguisticRelationshipNode
name_type_dict["WordNode"] = t.WordNode
name_type_dict["FeatureNode"] = t.FeatureNode
name_type_dict["AnchorNode"] = t.AnchorNode
name_type_dict["ProcedureNode"] = t.ProcedureNode
name_type_dict["BindLink"] = t.BindLink
name_type_dict["AGISIMPerceptNode"] = t.AGISIMPerceptNode
name_type_dict["AGISIMSelfNode"] = t.AGISIMSelfNode
name_type_dict["SchemaNode"] = t.SchemaNode
name_type_dict["IntensionalInheritanceLink"] = t.IntensionalInheritanceLink
name_type_dict["TimeNode"] = t.TimeNode
name_type_dict["WordSenseNode"] = t.WordSenseNode
name_type_dict["LinkGrammarRelationshipNode"] = t.LinkGrammarRelationshipNode
name_type_dict["SchemaEvaluationLink"] = t.SchemaEvaluationLink
name_type_dict["UnorderedLink"] = t.UnorderedLink
type_name_dict[t.CosenseLink] = "CosenseLink"
type_name_dict[t.TimeNode] = "TimeNode"
type_name_dict[t.AssociativeLink] = "AssociativeLink"
type_name_dict[t.Atom] = "Atom"
type_name_dict[t.SentenceNode] = "SentenceNode"
type_name_dict[t.ImplicationLink] = "ImplicationLink"
type_name_dict[t.SemanticRelationNode] = "SemanticRelationNode"
type_name_dict[t.DefinedFrameElementNode] = "DefinedFrameElementNode"
type_name_dict[t.NotLink] = "NotLink"
type_name_dict[t.PartOfSpeechNode] = "PartOfSpeechNode"
type_name_dict[t.GroundedPredicateNode] = "GroundedPredicateNode"
type_name_dict[t.SchemaNode] = "SchemaNode"
type_name_dict[t.ExtensionalEquivalenceLink] = "ExtensionalEquivalenceLink"
type_name_dict[t.AGISIMSoundNode] = "AGISIMSoundNode"
type_name_dict[t.SatisfyingSetLink] = "SatisfyingSetLink"
type_name_dict[t.AsymmetricHebbianLink] = "AsymmetricHebbianLink"
type_name_dict[t.ExistsLink] = "ExistsLink"
type_name_dict[t.AGISIMObjectPerceptNode] = "AGISIMObjectPerceptNode"
type_name_dict[t.EventualSequentialImplication] = "EventualSequentialImplication"
type_name_dict[t.ExtensionalSimilarityLink] = "ExtensionalSimilarityLink"
type_name_dict[t.SemeNode] = "SemeNode"
type_name_dict[t.CountLink] = "CountLink"
type_name_dict[t.VariableNode] = "VariableNode"
type_name_dict[t.MemberLink] = "MemberLink"
type_name_dict[t.InverseHebbianLink] = "InverseHebbianLink"
type_name_dict[t.EventualSequentialAND] = "EventualSequentialAND"
type_name_dict[t.Link] = "Link"
type_name_dict[t.TypedVariableLink] = "TypedVariableLink"
type_name_dict[t.UnknownObjectNode] = "UnknownObjectNode"
type_name_dict[t.LemmaNode] = "LemmaNode"
type_name_dict[t.EquivalenceLink] = "EquivalenceLink"
type_name_dict[t.VariableTypeNode] = "VariableTypeNode"
type_name_dict[t.SequentialAndLink] = "SequentialAndLink"
type_name_dict[t.SubsetLink] = "SubsetLink"
type_name_dict[t.HebbianLink] = "HebbianLink"
type_name_dict[t.SimultaneousAndLink] = "SimultaneousAndLink"
type_name_dict[t.OrderedLink] = "OrderedLink"
type_name_dict[t.ExtensionalImplicationLink] = "ExtensionalImplicationLink"
type_name_dict[t.DefinedLinguisticConceptNode] = "DefinedLinguisticConceptNode"
type_name_dict[t.FrameElementLink] = "FrameElementLink"
type_name_dict[t.AtTimeLink] = "AtTimeLink"
type_name_dict[t.CWPixelPerceptNode] = "CWPixelPerceptNode"
type_name_dict[t.DefinedFrameNode] = "DefinedFrameNode"
type_name_dict[t.LatestLink] = "LatestLink"
type_name_dict[t.LinkGrammarRelationshipNode] = "LinkGrammarRelationshipNode"
type_name_dict[t.AGISIMSmellNode] = "AGISIMSmellNode"
type_name_dict[t.SimilarityLink] = "SimilarityLink"
type_name_dict[t.NumberNode] = "NumberNode"
type_name_dict[t.PartOfSpeechLink] = "PartOfSpeechLink"
type_name_dict[t.SetLink] = "SetLink"
type_name_dict[t.IntensionalInheritanceLink] = "IntensionalInheritanceLink"
type_name_dict[t.ScholemLink] = "ScholemLink"
type_name_dict[t.AndLink] = "AndLink"
type_name_dict[t.AvatarNode] = "AvatarNode"
type_name_dict[t.PredictiveImplicationLink] = "PredictiveImplicationLink"
type_name_dict[t.ParseNode] = "ParseNode"
type_name_dict[t.TrueLink] = "TrueLink"
type_name_dict[t.CWColorNode] = "CWColorNode"
type_name_dict[t.TailPredictiveImplicationLink] = "TailPredictiveImplicationLink"
type_name_dict[t.AGIMSIMVisualPerceptNode] = "AGIMSIMVisualPerceptNode"
type_name_dict[t.ForAllLink] = "ForAllLink"
type_name_dict[t.AGISIMPixelPerceptNode] = "AGISIMPixelPerceptNode"
type_name_dict[t.AGISIMPolygonPerceptNode] = "AGISIMPolygonPerceptNode"
type_name_dict[t.StructureNode] = "StructureNode"
type_name_dict[t.InheritanceLink] = "InheritanceLink"
type_name_dict[t.AverageLink] = "AverageLink"
type_name_dict[t.GroundedSchemaNode] = "GroundedSchemaNode"
type_name_dict[t.ProcedureNode] = "ProcedureNode"
type_name_dict[t.PrepositionalRelationshipNode] = "PrepositionalRelationshipNode"
type_name_dict[t.HumanoidNode] = "HumanoidNode"
type_name_dict[t.OrLink] = "OrLink"
type_name_dict[t.MixedImplicationLink] = "MixedImplicationLink"
type_name_dict[t.ParseLink] = "ParseLink"
type_name_dict[t.SchemaExecutionLink] = "SchemaExecutionLink"
type_name_dict[t.WordInstanceLink] = "WordInstanceLink"
type_name_dict[t.Node] = "Node"
type_name_dict[t.ConceptNode] = "ConceptNode"
type_name_dict[t.LemmaLink] = "LemmaLink"
type_name_dict[t.FeatureNode] = "FeatureNode"
type_name_dict[t.SymmetricHebbianLink] = "SymmetricHebbianLink"
type_name_dict[t.FalseLink] = "FalseLink"
type_name_dict[t.ObjectNode] = "ObjectNode"
type_name_dict[t.AGISIMTasteNode] = "AGISIMTasteNode"
type_name_dict[t.ExecutionLink] = "ExecutionLink"
type_name_dict[t.IntensionalSimilarityLink] = "IntensionalSimilarityLink"
type_name_dict[t.IsAcceptableSecondArgLink] = "IsAcceptableSecondArgLink"
type_name_dict[t.UnorderedLink] = "UnorderedLink"
type_name_dict[t.AccessoryNode] = "AccessoryNode"
type_name_dict[t.LinkGrammarDisjunctNode] = "LinkGrammarDisjunctNode"
type_name_dict[t.SymmetricInverseHebbianLink] = "SymmetricInverseHebbianLink"
type_name_dict[t.WordInstanceNode] = "WordInstanceNode"
type_name_dict[t.PetNode] = "PetNode"
type_name_dict[t.SchemaEvaluationLink] = "SchemaEvaluationLink"
type_name_dict[t.AGISIMPerceptNode] = "AGISIMPerceptNode"
type_name_dict[t.BindLink] = "BindLink"
type_name_dict[t.WordSenseLink] = "WordSenseLink"
type_name_dict[t.DocumentNode] = "DocumentNode"
type_name_dict[t.AGISIMSelfNode] = "AGISIMSelfNode"
type_name_dict[t.HolonymLink] = "HolonymLink"
type_name_dict[t.ReferenceLink] = "ReferenceLink"
type_name_dict[t.WordSenseNode] = "WordSenseNode"
type_name_dict[t.AnchorNode] = "AnchorNode"
type_name_dict[t.GroundedProcedureNode] = "GroundedProcedureNode"
type_name_dict[t.ListLink] = "ListLink"
type_name_dict[t.EvaluationLink] = "EvaluationLink"
type_name_dict[t.DefinedLinguisticRelationshipNode] = "DefinedLinguisticRelationshipNode"
type_name_dict[t.SimultaneousEquivalenceLink] = "SimultaneousEquivalenceLink"
type_name_dict[t.HypotheticalLink] = "HypotheticalLink"
type_name_dict[t.ContextLink] = "ContextLink"
type_name_dict[t.PredicateNode] = "PredicateNode"
type_name_dict[t.ExecutionOutputLink] = "ExecutionOutputLink"
type_name_dict[t.FeatureLink] = "FeatureLink"
type_name_dict[t.WordNode] = "WordNode"

def is_a(child, parent):
    '''docstring for is_a''' 
    if isinstance(child, str) and isinstance(parent, str):
        return nx.has_path(types_graph, parent, child)
    else:
        return nx.has_path(types_graph, type_name_dict[parent], type_name_dict[child])

def type_to_name(t):
    '''docstring for typ''' 
    try:
        return type_name_dict[t]
    except KeyError:
        log.error("Unknown Atom type: %s, pls add related type infomation to file 'types_inheritance.py'"%t)
        raise KeyError
def name_to_type(name):
    try:
        return name_type_dict[name]
    except KeyError:
        log.error("Unknown Atom type: %s, pls add related type infomation to file 'types_inheritance.py'"%name)
        raise KeyError
    pass

__all__ = ["is_a", "type_name_dict", "name_type_dict", "type_to_name", "name_to_type"]

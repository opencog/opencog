##
# @file :types_inheritance.py
# @brief :relation of pln types, and map between "type_name" and type
# @author :Dingjie.Wang
# @version 1.0
# @date 2012-08-04
import networkx as nx
from opencog.atomspace import types as t
types_graph = nx.DiGraph()
name_to_type = { }
type_to_name = { }
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
types_graph.add_edge('Node','FWVariableNode')
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
name_to_type["EventualSequentialImplication"] = t.EventualSequentialImplication
name_to_type["FalseLink"] = t.FalseLink
name_to_type["DefinedFrameNode"] = t.DefinedFrameNode
name_to_type["InverseHebbianLink"] = t.InverseHebbianLink
name_to_type["GroundedSchemaNode"] = t.GroundedSchemaNode
name_to_type["Link"] = t.Link
name_to_type["EvaluationLink"] = t.EvaluationLink
name_to_type["TrueLink"] = t.TrueLink
name_to_type["NumberNode"] = t.NumberNode
name_to_type["HolonymLink"] = t.HolonymLink
name_to_type["HebbianLink"] = t.HebbianLink
name_to_type["TailPredictiveImplicationLink"] = t.TailPredictiveImplicationLink
name_to_type["SequentialAndLink"] = t.SequentialAndLink
name_to_type["EventualSequentialAND"] = t.EventualSequentialAND
name_to_type["SemanticRelationNode"] = t.SemanticRelationNode
name_to_type["AGISIMObjectPerceptNode"] = t.AGISIMObjectPerceptNode
name_to_type["StructureNode"] = t.StructureNode
name_to_type["CWPixelPerceptNode"] = t.CWPixelPerceptNode
name_to_type["Node"] = t.Node
name_to_type["ObjectNode"] = t.ObjectNode
name_to_type["LatestLink"] = t.LatestLink
name_to_type["SetLink"] = t.SetLink
name_to_type["PetNode"] = t.PetNode
name_to_type["AsymmetricHebbianLink"] = t.AsymmetricHebbianLink
name_to_type["SubsetLink"] = t.SubsetLink
name_to_type["ReferenceLink"] = t.ReferenceLink
name_to_type["SimultaneousEquivalenceLink"] = t.SimultaneousEquivalenceLink
name_to_type["DocumentNode"] = t.DocumentNode
name_to_type["WordInstanceLink"] = t.WordInstanceLink
name_to_type["SymmetricHebbianLink"] = t.SymmetricHebbianLink
name_to_type["VariableTypeNode"] = t.VariableTypeNode
name_to_type["ParseLink"] = t.ParseLink
name_to_type["IsAcceptableSecondArgLink"] = t.IsAcceptableSecondArgLink
name_to_type["SentenceNode"] = t.SentenceNode
name_to_type["PartOfSpeechNode"] = t.PartOfSpeechNode
name_to_type["LemmaNode"] = t.LemmaNode
name_to_type["MixedImplicationLink"] = t.MixedImplicationLink
name_to_type["ScholemLink"] = t.ScholemLink
name_to_type["VariableNode"] = t.VariableNode
name_to_type["AGIMSIMVisualPerceptNode"] = t.AGIMSIMVisualPerceptNode
name_to_type["NotLink"] = t.NotLink
name_to_type["CWColorNode"] = t.CWColorNode
name_to_type["ForAllLink"] = t.ForAllLink
name_to_type["AGISIMSmellNode"] = t.AGISIMSmellNode
name_to_type["UnknownObjectNode"] = t.UnknownObjectNode
name_to_type["DefinedLinguisticConceptNode"] = t.DefinedLinguisticConceptNode
name_to_type["FWVariableNode"] = t.FWVariableNode
name_to_type["PredictiveImplicationLink"] = t.PredictiveImplicationLink
name_to_type["SatisfyingSetLink"] = t.SatisfyingSetLink
name_to_type["ExtensionalSimilarityLink"] = t.ExtensionalSimilarityLink
name_to_type["OrLink"] = t.OrLink
name_to_type["ConceptNode"] = t.ConceptNode
name_to_type["FrameElementLink"] = t.FrameElementLink
name_to_type["ExtensionalImplicationLink"] = t.ExtensionalImplicationLink
name_to_type["AGISIMSoundNode"] = t.AGISIMSoundNode
name_to_type["AndLink"] = t.AndLink
name_to_type["OrderedLink"] = t.OrderedLink
name_to_type["SymmetricInverseHebbianLink"] = t.SymmetricInverseHebbianLink
name_to_type["GroundedPredicateNode"] = t.GroundedPredicateNode
name_to_type["PredicateNode"] = t.PredicateNode
name_to_type["WordSenseLink"] = t.WordSenseLink
name_to_type["MemberLink"] = t.MemberLink
name_to_type["AGISIMPixelPerceptNode"] = t.AGISIMPixelPerceptNode
name_to_type["Atom"] = t.Atom
name_to_type["IntensionalSimilarityLink"] = t.IntensionalSimilarityLink
name_to_type["PrepositionalRelationshipNode"] = t.PrepositionalRelationshipNode
name_to_type["ListLink"] = t.ListLink
name_to_type["SchemaExecutionLink"] = t.SchemaExecutionLink
name_to_type["AGISIMPolygonPerceptNode"] = t.AGISIMPolygonPerceptNode
name_to_type["ExecutionOutputLink"] = t.ExecutionOutputLink
name_to_type["WordInstanceNode"] = t.WordInstanceNode
name_to_type["ExtensionalEquivalenceLink"] = t.ExtensionalEquivalenceLink
name_to_type["AccessoryNode"] = t.AccessoryNode
name_to_type["AverageLink"] = t.AverageLink
name_to_type["AtTimeLink"] = t.AtTimeLink
name_to_type["SimilarityLink"] = t.SimilarityLink
name_to_type["GroundedProcedureNode"] = t.GroundedProcedureNode
name_to_type["CountLink"] = t.CountLink
name_to_type["CosenseLink"] = t.CosenseLink
name_to_type["ContextLink"] = t.ContextLink
name_to_type["LinkGrammarDisjunctNode"] = t.LinkGrammarDisjunctNode
name_to_type["ExecutionLink"] = t.ExecutionLink
name_to_type["HumanoidNode"] = t.HumanoidNode
name_to_type["EquivalenceLink"] = t.EquivalenceLink
name_to_type["AssociativeLink"] = t.AssociativeLink
name_to_type["AvatarNode"] = t.AvatarNode
name_to_type["LemmaLink"] = t.LemmaLink
name_to_type["TypedVariableLink"] = t.TypedVariableLink
name_to_type["ImplicationLink"] = t.ImplicationLink
name_to_type["AGISIMTasteNode"] = t.AGISIMTasteNode
name_to_type["SimultaneousAndLink"] = t.SimultaneousAndLink
name_to_type["ExistsLink"] = t.ExistsLink
name_to_type["HypotheticalLink"] = t.HypotheticalLink
name_to_type["InheritanceLink"] = t.InheritanceLink
name_to_type["PartOfSpeechLink"] = t.PartOfSpeechLink
name_to_type["DefinedFrameElementNode"] = t.DefinedFrameElementNode
name_to_type["FeatureLink"] = t.FeatureLink
name_to_type["ParseNode"] = t.ParseNode
name_to_type["SemeNode"] = t.SemeNode
name_to_type["DefinedLinguisticRelationshipNode"] = t.DefinedLinguisticRelationshipNode
name_to_type["WordNode"] = t.WordNode
name_to_type["FeatureNode"] = t.FeatureNode
name_to_type["AnchorNode"] = t.AnchorNode
name_to_type["ProcedureNode"] = t.ProcedureNode
name_to_type["BindLink"] = t.BindLink
name_to_type["AGISIMPerceptNode"] = t.AGISIMPerceptNode
name_to_type["AGISIMSelfNode"] = t.AGISIMSelfNode
name_to_type["SchemaNode"] = t.SchemaNode
name_to_type["IntensionalInheritanceLink"] = t.IntensionalInheritanceLink
name_to_type["TimeNode"] = t.TimeNode
name_to_type["WordSenseNode"] = t.WordSenseNode
name_to_type["LinkGrammarRelationshipNode"] = t.LinkGrammarRelationshipNode
name_to_type["SchemaEvaluationLink"] = t.SchemaEvaluationLink
name_to_type["UnorderedLink"] = t.UnorderedLink
type_to_name[t.CosenseLink] = "CosenseLink"
type_to_name[t.TimeNode] = "TimeNode"
type_to_name[t.AssociativeLink] = "AssociativeLink"
type_to_name[t.Atom] = "Atom"
type_to_name[t.SentenceNode] = "SentenceNode"
type_to_name[t.ImplicationLink] = "ImplicationLink"
type_to_name[t.SemanticRelationNode] = "SemanticRelationNode"
type_to_name[t.DefinedFrameElementNode] = "DefinedFrameElementNode"
type_to_name[t.NotLink] = "NotLink"
type_to_name[t.PartOfSpeechNode] = "PartOfSpeechNode"
type_to_name[t.GroundedPredicateNode] = "GroundedPredicateNode"
type_to_name[t.SchemaNode] = "SchemaNode"
type_to_name[t.ExtensionalEquivalenceLink] = "ExtensionalEquivalenceLink"
type_to_name[t.AGISIMSoundNode] = "AGISIMSoundNode"
type_to_name[t.SatisfyingSetLink] = "SatisfyingSetLink"
type_to_name[t.AsymmetricHebbianLink] = "AsymmetricHebbianLink"
type_to_name[t.ExistsLink] = "ExistsLink"
type_to_name[t.AGISIMObjectPerceptNode] = "AGISIMObjectPerceptNode"
type_to_name[t.EventualSequentialImplication] = "EventualSequentialImplication"
type_to_name[t.ExtensionalSimilarityLink] = "ExtensionalSimilarityLink"
type_to_name[t.SemeNode] = "SemeNode"
type_to_name[t.CountLink] = "CountLink"
type_to_name[t.VariableNode] = "VariableNode"
type_to_name[t.MemberLink] = "MemberLink"
type_to_name[t.InverseHebbianLink] = "InverseHebbianLink"
type_to_name[t.EventualSequentialAND] = "EventualSequentialAND"
type_to_name[t.Link] = "Link"
type_to_name[t.TypedVariableLink] = "TypedVariableLink"
type_to_name[t.UnknownObjectNode] = "UnknownObjectNode"
type_to_name[t.LemmaNode] = "LemmaNode"
type_to_name[t.EquivalenceLink] = "EquivalenceLink"
type_to_name[t.VariableTypeNode] = "VariableTypeNode"
type_to_name[t.SequentialAndLink] = "SequentialAndLink"
type_to_name[t.SubsetLink] = "SubsetLink"
type_to_name[t.HebbianLink] = "HebbianLink"
type_to_name[t.SimultaneousAndLink] = "SimultaneousAndLink"
type_to_name[t.OrderedLink] = "OrderedLink"
type_to_name[t.ExtensionalImplicationLink] = "ExtensionalImplicationLink"
type_to_name[t.DefinedLinguisticConceptNode] = "DefinedLinguisticConceptNode"
type_to_name[t.FrameElementLink] = "FrameElementLink"
type_to_name[t.AtTimeLink] = "AtTimeLink"
type_to_name[t.CWPixelPerceptNode] = "CWPixelPerceptNode"
type_to_name[t.DefinedFrameNode] = "DefinedFrameNode"
type_to_name[t.LatestLink] = "LatestLink"
type_to_name[t.LinkGrammarRelationshipNode] = "LinkGrammarRelationshipNode"
type_to_name[t.AGISIMSmellNode] = "AGISIMSmellNode"
type_to_name[t.SimilarityLink] = "SimilarityLink"
type_to_name[t.NumberNode] = "NumberNode"
type_to_name[t.FWVariableNode] = "FWVariableNode"
type_to_name[t.PartOfSpeechLink] = "PartOfSpeechLink"
type_to_name[t.SetLink] = "SetLink"
type_to_name[t.IntensionalInheritanceLink] = "IntensionalInheritanceLink"
type_to_name[t.ScholemLink] = "ScholemLink"
type_to_name[t.AndLink] = "AndLink"
type_to_name[t.AvatarNode] = "AvatarNode"
type_to_name[t.PredictiveImplicationLink] = "PredictiveImplicationLink"
type_to_name[t.ParseNode] = "ParseNode"
type_to_name[t.TrueLink] = "TrueLink"
type_to_name[t.CWColorNode] = "CWColorNode"
type_to_name[t.TailPredictiveImplicationLink] = "TailPredictiveImplicationLink"
type_to_name[t.AGIMSIMVisualPerceptNode] = "AGIMSIMVisualPerceptNode"
type_to_name[t.ForAllLink] = "ForAllLink"
type_to_name[t.AGISIMPixelPerceptNode] = "AGISIMPixelPerceptNode"
type_to_name[t.AGISIMPolygonPerceptNode] = "AGISIMPolygonPerceptNode"
type_to_name[t.StructureNode] = "StructureNode"
type_to_name[t.InheritanceLink] = "InheritanceLink"
type_to_name[t.AverageLink] = "AverageLink"
type_to_name[t.GroundedSchemaNode] = "GroundedSchemaNode"
type_to_name[t.ProcedureNode] = "ProcedureNode"
type_to_name[t.PrepositionalRelationshipNode] = "PrepositionalRelationshipNode"
type_to_name[t.HumanoidNode] = "HumanoidNode"
type_to_name[t.OrLink] = "OrLink"
type_to_name[t.MixedImplicationLink] = "MixedImplicationLink"
type_to_name[t.ParseLink] = "ParseLink"
type_to_name[t.SchemaExecutionLink] = "SchemaExecutionLink"
type_to_name[t.WordInstanceLink] = "WordInstanceLink"
type_to_name[t.Node] = "Node"
type_to_name[t.ConceptNode] = "ConceptNode"
type_to_name[t.LemmaLink] = "LemmaLink"
type_to_name[t.FeatureNode] = "FeatureNode"
type_to_name[t.SymmetricHebbianLink] = "SymmetricHebbianLink"
type_to_name[t.FalseLink] = "FalseLink"
type_to_name[t.ObjectNode] = "ObjectNode"
type_to_name[t.AGISIMTasteNode] = "AGISIMTasteNode"
type_to_name[t.ExecutionLink] = "ExecutionLink"
type_to_name[t.IntensionalSimilarityLink] = "IntensionalSimilarityLink"
type_to_name[t.IsAcceptableSecondArgLink] = "IsAcceptableSecondArgLink"
type_to_name[t.UnorderedLink] = "UnorderedLink"
type_to_name[t.AccessoryNode] = "AccessoryNode"
type_to_name[t.LinkGrammarDisjunctNode] = "LinkGrammarDisjunctNode"
type_to_name[t.SymmetricInverseHebbianLink] = "SymmetricInverseHebbianLink"
type_to_name[t.WordInstanceNode] = "WordInstanceNode"
type_to_name[t.PetNode] = "PetNode"
type_to_name[t.SchemaEvaluationLink] = "SchemaEvaluationLink"
type_to_name[t.AGISIMPerceptNode] = "AGISIMPerceptNode"
type_to_name[t.BindLink] = "BindLink"
type_to_name[t.WordSenseLink] = "WordSenseLink"
type_to_name[t.DocumentNode] = "DocumentNode"
type_to_name[t.AGISIMSelfNode] = "AGISIMSelfNode"
type_to_name[t.HolonymLink] = "HolonymLink"
type_to_name[t.ReferenceLink] = "ReferenceLink"
type_to_name[t.WordSenseNode] = "WordSenseNode"
type_to_name[t.AnchorNode] = "AnchorNode"
type_to_name[t.GroundedProcedureNode] = "GroundedProcedureNode"
type_to_name[t.ListLink] = "ListLink"
type_to_name[t.EvaluationLink] = "EvaluationLink"
type_to_name[t.DefinedLinguisticRelationshipNode] = "DefinedLinguisticRelationshipNode"
type_to_name[t.SimultaneousEquivalenceLink] = "SimultaneousEquivalenceLink"
type_to_name[t.HypotheticalLink] = "HypotheticalLink"
type_to_name[t.ContextLink] = "ContextLink"
type_to_name[t.PredicateNode] = "PredicateNode"
type_to_name[t.ExecutionOutputLink] = "ExecutionOutputLink"
type_to_name[t.FeatureLink] = "FeatureLink"
type_to_name[t.WordNode] = "WordNode"

def is_a(child, parent):
    '''docstring for is_a''' 
    if isinstance(child, str) and isinstance(parent, str):
        return nx.has_path(types_graph, parent, child)
    else:
        return nx.has_path(types_graph, type_to_name[parent], type_to_name[child])

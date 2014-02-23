;
; Test data for a stack-handling bug found by Samir
; The bug was stupid, and is unlikely to reappear, but
; this still seems like a good test case to have around.

(define (stv mean conf) (cog-new-stv mean conf))

; Input data
(InheritanceLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation")
   (WordSenseNode "#Manipulation")
)
(InheritanceLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Agent")
   (SemeNode "#Manipulation:Agent")
)
(InheritanceLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Depictive")
   (SemeNode "#Manipulation:Depictive")
)
(InheritanceLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Entity")
   (SemeNode "#Manipulation:Entity")
)

(FeatureLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation")
   (PredicateNode "grab@123_Manipulation_Agent")
)
(FeatureLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation")
   (PredicateNode "grab@123_Manipulation_Depictive")
)
(FeatureLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation")
   (PredicateNode "grab@123_Manipulation_Entity")
)


(EvaluationLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Agent")
   (ConceptNode "#you")
)
(EvaluationLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Depictive")
   (ConceptNode "#grab")
)
(EvaluationLink (stv 1.0 1.0)
   (PredicateNode "grab@123_Manipulation_Entity")
   (WordInstanceNode "ball@456")
)


; The implication to be run
(define (impy)

  (BindLink (stv 1 1)
   (ListLink
    (TypedVariableLink
     (VariableNode "$agent")
     (ListLink
      (VariableTypeNode "ConceptNode")
      (VariableTypeNode "WordInstanceNode")
      )
     )
    (TypedVariableLink
     (VariableNode "$framePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameAgentPredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameDepictivePredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$frameEntityPredicateNode")
     (VariableTypeNode "PredicateNode")
     )
    (TypedVariableLink
     (VariableNode "$targetEntity")
     (VariableTypeNode "WordInstanceNode")
     )
    )
   (ImplicationLink
    (AndLink
     (InheritanceLink
      (VariableNode "$framePredicateNode")
      (WordSenseNode "#Manipulation")
      )
     (InheritanceLink
      (VariableNode "$frameAgentPredicateNode")
      (SemeNode "#Manipulation:Agent")
      )
     (InheritanceLink
      (VariableNode "$frameDepictivePredicateNode")
      (SemeNode "#Manipulation:Depictive")
      )
     (InheritanceLink
      (VariableNode "$frameEntityPredicateNode")
      (SemeNode "#Manipulation:Entity")
      )
      
    
     (FeatureLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameEntityPredicateNode")
      )            
     (FeatureLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameAgentPredicateNode")
      )            
     (FeatureLink
      (VariableNode "$framePredicateNode")
      (VariableNode "$frameDepictivePredicateNode")
      )
    
     (EvaluationLink
      (VariableNode "$frameAgentPredicateNode")
      (VariableNode "$agent")
      )
     (EvaluationLink
      (VariableNode "$frameDepictivePredicateNode")
      (ConceptNode "#grab")
      )     
     (EvaluationLink
      (VariableNode "$frameEntityPredicateNode")
      (VariableNode "$targetEntity")
      )
    
     )
   
    (EvaluationLink
     (PredicateNode "grab")
     (ListLink
        (VariableNode "$targetEntity")
        (VariableNode "$frameDepictivePredicateNode")
      )
     )
    )
  )
)

; Running the implication should return only one answer!
; (cog-bind (impy))


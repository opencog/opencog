(ImplicationLink
 (AndLink
  (EvaluationLink 
   (PredicateNode "croaks")
   (VariableNode "$X")
  )
  (EvaluationLink     
    (PredicateNode "eats_flies")
    (VariableNode "$X")
  )
 )
 (InheritanceLink
  (VariableNode "$X")
  (ConceptNode "Frog")
 )
)

(ImplicationLink
 (AndLink
  (EvaluationLink 
   (PredicateNode "chirps")
   (VariableNode "$Y")
  )
  (EvaluationLink
   (PredicateNode "sings")
   (VariableNode "$Y")
  )
 )
 (InheritanceLink
  (VariableNode "$Y")
  (ConceptNode "Canary")
 )
)

(ImplicationLink
 (InheritanceLink 
  (VariableNode "$Z")
  (ConceptNode "Frog")
 )
 (InheritanceLink
  (VariableNode "$Z")
  (ConceptNode "green")
 )
)

(ImplicationLink
 (InheritanceLink 
  (VariableNode "$A")
  (ConceptNode "Canary")
 )
 (InheritanceLink
  (VariableNode "$A")
  (ConceptNode "yellow")
 )
)

;KB
(EvaluationLink
 (PredicateNode "croaks")
 (ConceptNode "Fritz")
)

(EvaluationLink
 (PredicateNode "chirps")
 (ConceptNode "Tweety")
)

(InheritanceLink
 (ConceptNode "Tweety")
 (ConceptNode "Yello")
)

(EvaluationLink 
 (PredicateNode "eats_flies")
 (ConceptNode "Tweety")
)

(EvaluationLink 
 (PredicateNode "eats_flies")
 (ConceptNode "Fritz")
)


; Define 2 dummy foralls to test it, one for ConceptNodes, the for
; PredicateNodes

(ForAllLink (stv 1 1)
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "ConceptNode"))
   (EvaluationLink
      (PredicateNode "is-concept")
      (VariableNode "$X")
   )
)

(ConceptNode "A")

(ForAllLink (stv 1 1)
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "PredicateNode"))
   (EvaluationLink
      (PredicateNode "is-predicate")
      (VariableNode "$X")
   )
)

(PredicateNode "P")

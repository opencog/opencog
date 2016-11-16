;; Define simple foralls to test universal rules

;; Simple ForAll involving concepts

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

;; Simple ForAll involving predicates

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

;; Simple ForAll involving concepts and predicates

(ForAllLink (stv 1 1)
   (VariableList
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode"))
      (TypedVariableLink
         (VariableNode "$Y")
         (TypeNode "PredicateNode")))
   (EvaluationLink
      (PredicateNode "are-concept-and-predicate")
      (ListLink
         (VariableNode "$X")
         (VariableNode "$Y"))
   )
)

;; Simple ForAll involving ImplicationLink

(ForAllLink (stv 1 1)
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "ConceptNode"))
   (ImplicationLink
      (EvaluationLink
         (PredicateNode "P")
         (VariableNode "$X"))
      (EvaluationLink
         (PredicateNode "P")
         (VariableNode "$X"))))

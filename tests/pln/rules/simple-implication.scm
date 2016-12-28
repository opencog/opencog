;; Define simple implications to test implication instantiation rules

(ImplicationScopeLink (stv 1 1)
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "ConceptNode"))
   (EvaluationLink (stv 0.2 0.9)
      (PredicateNode "P")
      (VariableNode "$X"))
   (EvaluationLink
      (PredicateNode "Q")
      (VariableNode "$X")))

(EvaluationLink (stv 1 1)
   (PredicateNode "P")
   (ConceptNode "A"))

(ImplicationScopeLink (stv 1 1)
   (VariableList
      (TypedVariableLink
         (VariableNode "$X")
         (TypeNode "ConceptNode"))
      (TypedVariableLink
         (VariableNode "$Y")
         (TypeNode "ConceptNode")))
   (EvaluationLink (stv 0.04 0.6)
      (PredicateNode "P")
      (ListLink
         (VariableNode "$X")
         (VariableNode "$Y")))
   (EvaluationLink
      (PredicateNode "Q")
      (ListLink
         (VariableNode "$Y")
         (VariableNode "$X"))))

(EvaluationLink (stv 1 1)
   (PredicateNode "P")
   (ListLink
      (ConceptNode "A")
      (ConceptNode "B")))

;; This one is to test the implication instantiation rule when the
;; precondition cannot be satisfied
(ImplicationScopeLink (stv 1 1)
   (TypedVariableLink
      (VariableNode "$X")
      (TypeNode "ConceptNode"))
   (EvaluationLink
      (PredicateNode "dummy-implicant")
      (VariableNode "$X"))
   (EvaluationLink
      (PredicateNode "dummy-implicand")
      (VariableNode "$X")))

;; This one is to test implicant distribution

(ImplicationLink
   (PredicateNode "P")
   (PredicateNode "Q"))

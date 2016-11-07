;; Define implication with extra variable in the implicand

(ImplicationScopeLink (stv 1 1)
   (VariableList
      (TypedVariableLink
        (VariableNode "$X")
        (TypeNode "ConceptNode"))
     (TypedVariableLink
        (VariableNode "$Y")
        (TypeNode "ConceptNode")))
   (EvaluationLink (stv 0.2 0.9)
      (PredicateNode "P")
      (VariableNode "$X"))
   (EvaluationLink
      (PredicateNode "Q")
      (List
         (VariableNode "$X")
         (VariableNode "$Y"))))

(EvaluationLink (stv 1 1)
   (PredicateNode "P")
   (ConceptNode "A"))

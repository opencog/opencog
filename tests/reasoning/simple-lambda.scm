;; Simple examples for LambdaLink for testing Lambda distribution rules

(LambdaLink (stv 0.6 0.3)
   (VariableNode "$X")
   (AndLink
      (EvaluationLink
         (PredicateNode "P")
         (VariableNode "$X"))
      (EvaluationLink
         (PredicateNode "Q")
         (VariableNode "$X"))))

(LambdaLink
   (VariableNode "$X")
   (EvaluationLink (stv 0.4 0.7)
      (PredicateNode "P")
      (ConceptNode "A")))

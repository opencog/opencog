(ImplicationLink (stv 0.900000 0.111111)
  (AndLink
      (ConceptNode "Thomas" (stv 0.010000 0.555556))
      (ConceptNode "Train" (stv 0.010000 0.555556))
  )
  (PredicateNode "goesOn")
)
(EvaluationLink
  (PredicateNode "rules")
  (ListLink
    (ConceptNode "AndCreationRule")
    (ConceptNode "ModusPonensRule<ImplicationLink>")
  )
)
(EvaluationLink
  (PredicateNode "query")
  (ListLink (PredicateNode "goesOn"))
)


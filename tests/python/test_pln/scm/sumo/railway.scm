(MemberLink (stv 0.900000 0.111111)
  (ConceptNode "Train1" (stv 0.010000 0.555556))
  (ConceptNode "Trains" (stv 0.010000 0.555556))
)
(ImplicationLink (stv 0.900000 0.111111)
  (MemberLink (stv 0.900000 0.111111)
    (ConceptNode "Train1" (stv 0.010000 0.555556))
    (ConceptNode "Trains" (stv 0.010000 0.555556))
  )
  (MemberLink
    (ConceptNode "Train1" (stv 0.010000 0.555556))
    (ConceptNode "CuteThings" (stv 0.010000 0.555556))
  )
)
(EvaluationLink
  (PredicateNode "rules")
  (ListLink
    (ConceptNode "ModusPonensRule<ImplicationLink>")
  )
)
(EvaluationLink
  (PredicateNode "query")
  (ListLink
    (MemberLink
      (ConceptNode "Train1")
      (ConceptNode "CuteThings")
    )
  )
)

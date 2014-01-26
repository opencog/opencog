(MemberLink (stv 0.900000 0.111111)
  (ConceptNode "Thomas")
  (ConceptNode "Train")
)

(MemberLink (stv 0.900000 0.111111)
  (ConceptNode "Frankston Line")
  (ConceptNode "Track")
)

(ImplicationLink (stv 0.900000 0.111111)
  (AndLink
    (MemberLink
      (VariableNode "?TRAIN")
      (ConceptNode "Train")
    )
    (MemberLink
      (VariableNode "?TRACK")
      (ConceptNode "Track")
    )
  )
  (EvaluationLink
    (PredicateNode "goesOn")
    (ListLink
        (VariableNode "?TRAIN")
        (VariableNode "?TRACK")
    )
  )
)


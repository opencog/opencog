(define wakeUp
  (EvaluationLink (stv 1.0 1)
     (PredicateNode "wakeUp")
  )
)

(define eatBreakfast
  (EvaluationLink (stv 1.0 1)
     (PredicateNode "eatBreakfast")
  )
)

(define leaveHouse
  (EvaluationLink (stv 1.0 1)
     (PredicateNode "leaveHouse")
  )
)

(BeforeLink (stv 1.0 1)
  wakeUp
  eatBreakfast
)

(BeforeLink (stv 1.0 1)
  eatBreakfast
  leaveHouse
)

(define wakeUpBeforeLeaveHouse
  (BeforeLink
    wakeUp
    leaveHouse
  )
)

(EvaluationLink (PredicateNode "query") (ListLink wakeUpBeforeLeaveHouse))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "BeforeLinkTransitivityRule")))


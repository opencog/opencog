(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))
(define or_ab (OrLink a b))

(EvaluationLink (PredicateNode "query") (ListLink or_ab))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "OrCreationRule")))


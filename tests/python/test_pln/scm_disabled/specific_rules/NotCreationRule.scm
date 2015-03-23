; test of NotRule
(define A (ConceptNode "A" (stv 0.1 0.3)))
(define NotA (NotLink A))

(EvaluationLink (PredicateNode "query") (ListLink NotA))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "NotCreationRule")))


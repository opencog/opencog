(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))
(define and_ab (AndLink a b))

(EvaluationLink (PredicateNode "query") (ListLink and_ab))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "AndCreationRule<2>")))


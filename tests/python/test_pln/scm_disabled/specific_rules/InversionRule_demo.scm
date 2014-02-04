(define a (ConceptNode "A" (stv 0.3 0.8)))
(define b (ConceptNode "B" (stv 0.1 0.85)))
(define inh_ab (InheritanceLink a b (stv 0.2 0.78)))

; target for InversionRule
(define inh_ba (InheritanceLink b a (av 1 1 0)))

(EvaluationLink (PredicateNode "query") (ListLink inh_ba))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "InversionRule")))


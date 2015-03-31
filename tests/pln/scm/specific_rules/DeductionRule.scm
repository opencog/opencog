; The formula for deduction uses at least part of every Node's TV.
(define a (ConceptNode "A" (stv 0.8 0.8)))
(define b (ConceptNode "B" (stv 0.2 0.65)))
(define c (ConceptNode "C" (stv 0.47 0.82)))
(define imp_ab (InheritanceLink a b (stv 0.5 0.75)))
(define imp_bc (InheritanceLink b c (stv 0.6 0.7)))

(define imp_ac (InheritanceLink a c))

(EvaluationLink (PredicateNode "query") (ListLink imp_ac))
(EvaluationLink (PredicateNode "rules") (ListLink (ConceptNode "DeductionRule<InheritanceLink>")))


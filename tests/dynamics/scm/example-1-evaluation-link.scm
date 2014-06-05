; Example #1: EvaluationLink
; ECAN_MAX_SPREAD_PERCENTAGE = 0.60

(define predicate (PredicateNode "smokes"))
(define concept (ConceptNode "Anna" (stv 0.1 1)))
(define list-link (ListLink concept))

(define evaluation
	(EvaluationLink (stv 1.0 1.0)
	    predicate
	    list-link))

(define case-1
	(cog-set-av! predicate (av 1000 0 0)))

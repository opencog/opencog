(define true (stv 1.0 0.1))
(define false (stv 0.0 0.1))

(define A (ConceptNode "A"))
(define B (ConceptNode "B"))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(MemberLink (ConceptNode "1") a true)
		(MemberLink (ConceptNode "2") a true)
		(MemberLink (ConceptNode "3") a true)
		(MemberLink (ConceptNode "4") a true)

		(MemberLink (ConceptNode "1") b true)
		(MemberLink (ConceptNode "2") b false)
		(MemberLink (ConceptNode "3") b true)
		(MemberLink (ConceptNode "4") b false)
		(MemberLink (ConceptNode "42") b true)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "AndBulkEvaluationRule<2>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(MemberLink (ConceptNode "1") a true)
		(MemberLink (ConceptNode "2") a true)
		(MemberLink (ConceptNode "3") a true)
		(MemberLink (ConceptNode "4") a true)

		(MemberLink (ConceptNode "1") b true)
		(MemberLink (ConceptNode "2") b false)
		(MemberLink (ConceptNode "3") b true)
		(MemberLink (ConceptNode "4") b false)
		(MemberLink (ConceptNode "42") b true)
		(AndLink a b (stv 0.252525 1))
	)
)


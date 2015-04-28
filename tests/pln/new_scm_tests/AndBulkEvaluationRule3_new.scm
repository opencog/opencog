(define true (stv 1.0 0.1))
(define false (stv 0.0 0.1))

(define A (ConceptNode "A"))
(define B (ConceptNode "B"))
(define C (ConceptNode "C"))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(MemberLink (ConceptNode "1") A true)
		(MemberLink (ConceptNode "2") A true)
		(MemberLink (ConceptNode "3") A true)
		(MemberLink (ConceptNode "4") A true)

		(MemberLink (ConceptNode "1") B true)
		(MemberLink (ConceptNode "2") B false)
		(MemberLink (ConceptNode "3") B true)
		(MemberLink (ConceptNode "4") B false)
		(MemberLink (ConceptNode "42") B true)

		(MemberLink (ConceptNode "1") C true)
		(MemberLink (ConceptNode "2") C true)
		(MemberLink (ConceptNode "3") C true)
		(MemberLink (ConceptNode "4") C true)
		(MemberLink (ConceptNode "tigerzrkewl") C true)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "AndBulkEvaluationRule<3>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(MemberLink (ConceptNode "1") A true)
		(MemberLink (ConceptNode "2") A true)
		(MemberLink (ConceptNode "3") A true)
		(MemberLink (ConceptNode "4") A true)

		(MemberLink (ConceptNode "1") B true)
		(MemberLink (ConceptNode "2") B false)
		(MemberLink (ConceptNode "3") B true)
		(MemberLink (ConceptNode "4") B false)
		(MemberLink (ConceptNode "42") B true)

		(MemberLink (ConceptNode "1") C true)
		(MemberLink (ConceptNode "2") C true)
		(MemberLink (ConceptNode "3") C true)
		(MemberLink (ConceptNode "4") C true)
		(MemberLink (ConceptNode "tigerzrkewl") C true)
		(AndLink A B C)
	)
)


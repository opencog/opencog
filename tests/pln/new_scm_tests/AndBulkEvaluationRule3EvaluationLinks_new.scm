(define true (stv 1.0 0.1))
(define false (stv 0.0 0.1))

(define A (PredicateNode "A"))
(define B (PredicateNode "B"))
(define C (PredicateNode "B"))

(define one (ConceptNode "1"))
(define two (ConceptNode "2"))
(define three (ConceptNode "3"))
(define four (ConceptNode "4"))

(define arg1 (VariableNode "$var1"))
(define arg2 (VariableNode "$var2"))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(EvaluationLink A (ListLink one two) true)
		(EvaluationLink A (ListLink one four) true)
		(EvaluationLink A (ListLink three four) true)

		(EvaluationLink B (ListLink one two) true)
		(EvaluationLink B (ListLink one four) true)
		(EvaluationLink B (ListLink three four) false)
		(EvaluationLink B (ListLink four four) true)

		(EvaluationLink C (ListLink one two) true)
		(EvaluationLink C (ListLink two one) true)
		(EvaluationLink C (ListLink four one) true)
		(EvaluationLink C (ListLink three four) true)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "AndBulkEvaluationRule<3>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(AndLink
			(EvaluationLink A (ListLink one two) true)
			(EvaluationLink A (ListLink one four) true)
			(EvaluationLink A (ListLink three four) true)

			(EvaluationLink B (ListLink one two) true)
			(EvaluationLink B (ListLink one four) true)
			(EvaluationLink B (ListLink three four) false)
			(EvaluationLink B (ListLink four four) true)

			(EvaluationLink C (ListLink one two) true)
			(EvaluationLink C (ListLink two one) true)
			(EvaluationLink C (ListLink four one) true)
			(EvaluationLink C (ListLink three four) true)
			(EvaluationLink A (ListLink arg1 arg2))
			(EvaluationLink B (ListLink arg1 arg2))
			(EvaluationLink C (ListLink arg1 arg2))
		)
	)
)

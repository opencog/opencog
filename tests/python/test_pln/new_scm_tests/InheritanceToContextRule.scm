; "Ben is competent in doing mathematics.
; "Ben is not competent in juggling."

(define ben (ConceptNode "Ben" (stv 0.01 1)))
(define competent (ConceptNode "competent" (stv 0.01 1)))
(define maths (ConceptNode "doing_mathematics" (stv 0.01 1)))
(define juggling (ConceptNode "juggling" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
		(InheritanceLink (stv 0.5 1)
		    (AndLink ben maths)
		    (AndLink competent maths) ; ConceptNodes are switched here
		)
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "InheritanceToContextRule")
	)
)

(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs")
	(ListLink
		ben
		competent
		maths
		(AndLink ben maths)
		(AndLink competent maths)
		(ContextLink (stv 0.5 1)
		    maths
		    (InheritanceLink ben competent)
		)
    )
)

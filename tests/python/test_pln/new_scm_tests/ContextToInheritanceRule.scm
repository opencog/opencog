; "Ben is competent in doing mathematics.
; "Ben is not competent in juggling."

(define ben (ConceptNode "Ben" (stv 0.01 1)))
(define competent (ConceptNode "competent" (stv 0.01 1)))
(define maths (ConceptNode "doing_mathematics" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs")
	(ListLink
		(ContextLink (stv 0.5 1)
		    maths
		    (InheritanceLink ben competent)
		)
    )
)

(EvaluationLink (PredicateNode "rules")
	(ListLink
		(ConceptNode "ContextToInheritanceRule")
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
		(InheritanceLink (stv 0.5 1)
		    (AndLink maths ben) ; ConceptNodes are switched here
		    (AndLink maths competent)
		)
    )
)

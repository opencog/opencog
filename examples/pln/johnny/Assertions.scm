(ConceptNode "Johnny@444") (stv 0.99 1)

(ConceptNode "Sweets@555") (stv 0.99 1)

(ConceptNode "day@222") (stv 0.99 1)

(ConceptNode "he@345") (stv 0.99 1)

(ConceptNode "himself@456") (stv 0.99 1)

(ConceptNode "risk@44") (stv 0.99 1)

(ConceptNode "diabetes@66") (stv 0.99 1)

(ImplicationLink (stv 0.99 1)
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555")))))
	(EvaluationLink
		(PredicateNode "at@123")
		(ListLink
			(EvaluationLink
				(PredicateNode "placing@234")
				(ListLink
					(ConceptNode "he@345")
					(ConceptNode "himself@456")))
			(ConceptNode "risk@44"))))

(EvaluationLink (stv 0.99 1)
	(PredicateNode "for@55")
	(ListLink
		(ConceptNode "risk@44")
		(ConceptNode "diabetes@66")))

(NotLink (stv 0.99 1)
	(EvaluationLink
		(PredicateNode "every@111")
        (ListLink
			(ConceptNode "day@222")
			(EvaluationLink
				(PredicateNode "eating@333")
				(ListLink
					(ConceptNode "Johnny@444")
					(ConceptNode "Sweets@555"))))))

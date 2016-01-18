(define spanishPeople (ConceptNode "Spanish" (stv 0.01 1)))
(define italianPeople (ConceptNode "Italian" (stv 0.01 1)))
(define greekPeople (ConceptNode "Greek" (stv 0.01 1)))

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(SimilarityLink (stv 0.5 1) spanishPeople italianPeople)
		(SimilarityLink (stv 0.5 1) italianPeople greekPeople)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "TransitiveSimilarityRule<SimilarityLink>")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		spanishPeople
		italianPeople
		greekPeople
		(SimilarityLink (stv 0.5 1) spanishPeople italianPeople)
		(SimilarityLink (stv 0.5 1) italianPeople greekPeople)
		(SimilarityLink (stv 0.286643 1.000000) spanishPeople greekPeople)
	)
)



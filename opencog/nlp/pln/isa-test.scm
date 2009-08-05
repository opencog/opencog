;
; isa-test.scm
;
; Test input data for isa pln deduction
;
(EvaluationLink (stv 1 1)
	(DefinedLinguisticRelationshipNode "isa")
	(ListLink
		(WordInstanceNode "cat@123")
		(WordInstanceNode "Becky@456")
	)
)

(EvaluationLink (stv 1 1)
	(DefinedLinguisticRelationshipNode "isa")
	(ListLink
		(WordInstanceNode "animal@123")
		(WordInstanceNode "cat@123")
	)
)

(define q
	(EvaluationLink 
		(DefinedLinguisticRelationshipNode "isa")
		(ListLink
			(WordInstanceNode "animal@123")
			(WordInstanceNode "Becky@456")
		)
	)
)

(pln-bc q 300)

;
; query-test.scm
;
; Simple, hand-administered test for the query functions.
;

(EvaluationLink (stv 1 1) 
	(DefinedLinguisticRelationshipNode "capital_of")
	(ListLink
		(WordInstanceNode "France@53dd8b72-5a76-4eeb-a711-0b923bbdd8bb")
		(WordInstanceNode "Paris@453")
	)
)

(InheritanceLink
	(WordInstanceNode "what@19503ca4-c0b2-4c92-9b08-e3109ab97ebf")
	(DefinedLinguisticConceptNode "what")
)

(define quest 
	(EvaluationLink (stv 1 1) 
		(DefinedLinguisticRelationshipNode "capital_of")
		(ListLink
			(WordInstanceNode "France@53dd8b72-5a76-4eeb-a711-0b923bbdd8bb")
			(WordInstanceNode "what@19503ca4-c0b2-4c92-9b08-e3109ab97ebf")
		)
	)
)

(find-wh-words quest)

(define impl (make-triple-question quest))

(cog-ad-hoc "do-varscope" impl)

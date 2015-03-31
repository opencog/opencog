; define concepts. Note that PLN ignores atoms with (0,0) truth values.
(define lion (ConceptNode "Lion" (stv 0.5 0.999)))
(define tiger (ConceptNode "Tiger" (stv 0.5 0.999)))
; define properties
(define mammal (ConceptNode "Mammal" (stv 0.01 0.999)))
; define default confidence
(define dc 0.5)

; How likely various things are to be a Lion
(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(AttractionLink (stv 1 dc) mammal lion)
		(AttractionLink (stv 1 dc) mammal tiger)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "IntensionalLinkEvaluationRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "1")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(AttractionLink (stv 1 dc) mammal lion)
		(AttractionLink (stv 1 dc) mammal tiger)
		(IntensionalInheritanceLink (stv 1.000000 0.001248) lion tiger)
	)
)

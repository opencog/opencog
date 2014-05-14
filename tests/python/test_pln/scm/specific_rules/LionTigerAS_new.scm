;scm
;
; LionTigerKB.scm - contains Subset links between concept Lion (resp. Tiger)
; and properties like Mammal, Carnivore, Striped, Predator, FacingExtinction
; as well as the negated versions of those properties.
; It is used to test IntensionalInhertianceRule
; "mammal" is a distinctive property of "Lion" if:
;   x being a mammal makes it more likely x is a lion.
;   i.e. P(x is a lion | x is a mammal) > P(x is a lion | x is not a mammal)
;
; define concepts. Note that PLN ignores atoms with (0,0) truth values.
(define lion (ConceptNode "Lion" (stv 0.5 0.999)))
(define tiger (ConceptNode "Tiger" (stv 0.5 0.999)))
; define properties
(define mammal (ConceptNode "Mammal" (stv 0.01 0.999)))
(define carnivore (ConceptNode "Carnivore" (stv 0.01 0.999)))
(define striped (ConceptNode "Striped" (stv 0.01 0.999)))
(define predator (ConceptNode "Predator" (stv 0.01 0.999)))
(define facingExtinction (ConceptNode "FacingExtinction" (stv 0.01 0.999)))
; define default confidence
(define dc 0.5)

(EvaluationLink (PredicateNode "inputs") 
	(ListLink 
		(SubsetLink (stv 1 dc) mammal lion)
		(SubsetLink (stv 1 dc) carnivore lion)
		(SubsetLink (stv 0 dc) striped lion)
		(SubsetLink (stv 1 dc) predator lion)
		(SubsetLink (stv 0.8 dc) facingExtinction lion)

		(SubsetLink (stv 0.2 dc) (NotLink mammal) lion)
		(SubsetLink (stv 0.1 dc) (NotLink carnivore) lion)
		(SubsetLink (stv 0.17 dc) (NotLink striped) lion)
		(SubsetLink (stv 0.3 dc) (NotLink predator) lion)
		(SubsetLink (stv 0.4 dc) (NotLink facingExtinction) lion)

		(SubsetLink (stv 1 dc) mammal tiger)
		(SubsetLink (stv 1 dc) carnivore tiger)
		(SubsetLink (stv 0.9 dc) striped tiger)
		(SubsetLink (stv 1 dc) predator tiger)
		(SubsetLink (stv 1 dc) facingExtinction tiger)

		(SubsetLink (stv 0.2 dc) (NotLink mammal) tiger)
		(SubsetLink (stv 0.1 dc) (NotLink carnivore) tiger)
		(SubsetLink (stv 0.15 dc) (NotLink striped) tiger)
		(SubsetLink (stv 0.3 dc) (NotLink predator) tiger)
		(SubsetLink (stv 0.37 dc) (NotLink facingExtinction) tiger)
	)
)
(EvaluationLink (PredicateNode "rules") 
	(ListLink 
		(ConceptNode "IntensionalLinkEvaluationRule")
		(ConceptNode "AttractionRule")
	)
)
(EvaluationLink (PredicateNode "forwardSteps")
	(ListLink
		(NumberNode "10")
	)
)

(EvaluationLink (PredicateNode "outputs") 
	(ListLink 
		(SubsetLink (stv 1 dc) mammal lion)
		(SubsetLink (stv 1 dc) carnivore lion)
		(SubsetLink (stv 0 dc) striped lion)
		(SubsetLink (stv 1 dc) predator lion)
		(SubsetLink (stv 0.8 dc) facingExtinction lion)

		(SubsetLink (stv 0.2 dc) (NotLink mammal) lion)
		(SubsetLink (stv 0.1 dc) (NotLink carnivore) lion)
		(SubsetLink (stv 0.17 dc) (NotLink striped) lion)
		(SubsetLink (stv 0.3 dc) (NotLink predator) lion)
		(SubsetLink (stv 0.4 dc) (NotLink facingExtinction) lion)

		(SubsetLink (stv 1 dc) mammal tiger)
		(SubsetLink (stv 1 dc) carnivore tiger)
		(SubsetLink (stv 0.9 dc) striped tiger)
		(SubsetLink (stv 1 dc) predator tiger)
		(SubsetLink (stv 1 dc) facingExtinction tiger)

		(SubsetLink (stv 0.2 dc) (NotLink mammal) tiger)
		(SubsetLink (stv 0.1 dc) (NotLink carnivore) tiger)
		(SubsetLink (stv 0.15 dc) (NotLink striped) tiger)
		(SubsetLink (stv 0.3 dc) (NotLink predator) tiger)
		(SubsetLink (stv 0.37 dc) (NotLink facingExtinction) tiger)
		(IntensionalInheritanceLink lion tiger)
	)
)

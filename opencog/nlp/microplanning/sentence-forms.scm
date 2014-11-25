; -----------------------------------------------------------------------
; microplanning-init -- Initialization code for microplanning
;
; Wrapping some of the persisitance atoms required for microplanning in a
; function so that they can be dynamically created without being affected
; by (clear).
;
; (VariableNode "MicroplanningWildcardMarker") is a unique node that
; microplanning recognizes as "match to any type of atom", whereas all the
; other links and nodes must match to the exact same type of links or
; nodes, with some additional restrictions on the name.
;
; "MicroplanningAnyNameMarker" means don't care about the name of the node
; (but, as mentioned, the node type still needs to match).
;
; "MicroplanningVerbMarker" means the name of the node must have a
; corresponding WordInstanceNode whose POS is a verb.
;
(define (microplanning-init)
	(InheritanceLink
		(ConceptNode "DeclarativeUtterance")
		(OrLink
			(EvaluationLink
				(PredicateNode "MicroplanningVerbMarker")
				(VariableNode "MicroplanningWildcardMarker")
			)
		)
	)
	
	(InheritanceLink
		(ConceptNode "InterrogativeUtterance")
		(OrLink
			(EvaluationLink
				(PredicateNode "MicroplanningVerbMarker")
				(VariableNode "MicroplanningWildcardMarker")
			)
			(EvaluationLink
				(VariableNode "MicroplanningAnyNameMarker")
				(ListLink
					(ConceptNode "MicroplanningAnyNameMarker")
				)
			)
		)
	)
)


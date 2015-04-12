;
; Basic unit testing for differen was of nesting evaluatable links.
;
(use-modules (opencog))
(use-modules (opencog query))

;;; Populate the atomspace with a bouquet of arcs
(AssociativeLink
	(ConceptNode "idea one")
	(ConceptNode "idea one")
)

(AssociativeLink
	(ConceptNode "idea one")
	(ConceptNode "idea two")
)

(AssociativeLink
	(ConceptNode "idea two")
	(ConceptNode "idea three")
)

(AssociativeLink
	(ConceptNode "idea three")
	(ConceptNode "idea four")
)

(AssociativeLink
	(ConceptNode "idea four")
	(ConceptNode "idea five")
)

(AssociativeLink
	(ConceptNode "idea two")
	(ConceptNode "idea one")
)

(AssociativeLink
	(ConceptNode "idea three")
	(ConceptNode "idea one")
)

(AssociativeLink
	(ConceptNode "idea four")
	(ConceptNode "idea one")
)

(AssociativeLink
	(ConceptNode "idea five")
	(ConceptNode "idea one")
)

(AssociativeLink
	(ConceptNode "idea one")
	(ConceptNode "idea three")
)

(AssociativeLink
	(ConceptNode "idea one")
	(ConceptNode "idea four")
)

(AssociativeLink
	(ConceptNode "idea one")
	(ConceptNode "idea five")
)

;;; Explore the connectivity of the graph
(define (five-arcs)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(AndLink
				(AssociativeLink
					(ConceptNode "idea one")
					(VariableNode "$x")
				)
				(AssociativeLink
					(VariableNode "$x")
					(ConceptNode "idea one")
				)
			)
			(VariableNode "$x")
		)
	)
)

;
; Basic unit testing for differen was of nesting evaluatable links.
;
(use-modules (opencog))
(use-modules (opencog query))

;;; Populate the atomspace with a cover of a directed bouquet of four circles
;;; All nodes are equivalent to one, and there are directed arrows
;;; in the node order...
(AssociativeLink (ConceptNode "idea one") (ConceptNode "idea one"))

(AssociativeLink (ConceptNode "idea one") (ConceptNode "idea two"))
(AssociativeLink (ConceptNode "idea two") (ConceptNode "idea one"))

(AssociativeLink (ConceptNode "idea two") (ConceptNode "idea three"))
(AssociativeLink (ConceptNode "idea three") (ConceptNode "idea one"))

(AssociativeLink (ConceptNode "idea three") (ConceptNode "idea four"))
(AssociativeLink (ConceptNode "idea four") (ConceptNode "idea one"))

(AssociativeLink (ConceptNode "idea four") (ConceptNode "idea five"))
(AssociativeLink (ConceptNode "idea five") (ConceptNode "idea one"))

(AssociativeLink (ConceptNode "idea one") (ConceptNode "idea three"))
(AssociativeLink (ConceptNode "idea one") (ConceptNode "idea four"))
(AssociativeLink (ConceptNode "idea one") (ConceptNode "idea five"))

;;; The recurring core of all the tests

(define  one->x
	(AssociativeLink
		(ConceptNode "idea one")
		(VariableNode "$x")
	)
)
(define x->one
	(AssociativeLink
		(VariableNode "$x")
		(ConceptNode "idea one")
	)
)

;;; Explore the connectivity of the graph
(define (five-arcs)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(AndLink one->x x->one
			)
			(VariableNode "$x")
		)
	)
)

(define (one-arc)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(AndLink one->x x->one
				(EqualLink (VariableNode "$x") (ConceptNode "idea one"))
			)
			(VariableNode "$x")
		)
	)
)

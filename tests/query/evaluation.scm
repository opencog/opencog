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

(define (wrapper core)
	(BindLink
		(VariableNode "$x")
		(ImplicationLink
			(AndLink core)
			(VariableNode "$x")
		)
	)
)

;;; Explore the connectivity of the graph

;; All five nodes are bi-connected.
(define (five-arcs)
	(wrapper (list one->x x->one))
)

;; Reject all but one arc
(define (one-arc-one)
	(wrapper
		(list one->x x->one
			(EqualLink (VariableNode "$x") (ConceptNode "idea one"))
		)
	)
)

(define (one-arc-three)
	(wrapper
		(list one->x x->one
			(EqualLink (VariableNode "$x") (ConceptNode "idea three"))
		)
	)
)

;;; conflict thus not satisfiable $x cannot be 3 and 4 at the same time
(define (zero-arcs)
	(wrapper
		(list one->x x->one
			(EqualLink (VariableNode "$x") (ConceptNode "idea three"))
			(EqualLink (VariableNode "$x") (ConceptNode "idea four"))
		)
	)
)

;; reject node three only; of the five, four remain
(define (four-arcs)
	(wrapper
		(list one->x x->one
			(AbsentLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea three"))
			)
		)
	)
)

;; reject three nodes; of the five, two remain
(define (two-arcs)
	(wrapper
		(list one->x x->one
			(AbsentLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea three"))
			)
			(AbsentLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea four"))
			)
			(AbsentLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea five"))
			)
		)
	)
)

;;;; -----------------------------------------------------------------
;;; Boolean logic connectives!

;; reject node one only; of the five, four remain
(define (four-not)
	(wrapper
		(list one->x x->one
			(NotLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea one"))
			)
		)
	)
)

;; accept either of two.
(define (two-or)
	(wrapper
		(list one->x x->one
			(OrLink
				(EqualLink (VariableNode "$x") (ConceptNode "idea one"))
				(EqualLink (VariableNode "$x") (ConceptNode "idea two"))
			)
		)
	)
)

;; accept neither of two. (accept three)
(define (three-nor)
	(wrapper
		(list one->x x->one
			(NotLink
				(OrLink
					(EqualLink (VariableNode "$x") (ConceptNode "idea one"))
					(EqualLink (VariableNode "$x") (ConceptNode "idea two"))
				)
			)
		)
	)
)

;; accept any of the first two;
(define (two-fancy)
	(wrapper
		(list one->x x->one
			(AndLink
				(NotLink
					(EqualLink (VariableNode "$x") (ConceptNode "idea three"))
				)
				(NotLink
					(OrLink
						(EqualLink (VariableNode "$x") (ConceptNode "idea four"))
						(EqualLink (VariableNode "$x") (ConceptNode "idea five"))
					)
				)
			)
		)
	)
)

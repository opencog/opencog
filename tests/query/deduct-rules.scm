;;
;; deduct-rules.scm
;;
;; Deduction rules for Einstein puzzle.
;;
;; The rules here are written in a fashion as close as possible to
;; 'ordinary' common-sense deductive rules.  In particular, they are
;; not written to predispose the problem into a 5x5 solution grid
;; (although this is what they eventually must lead to). In other
;; words, there is no effort made to make this teh most "efficient"
;; possible set of rules; instead, they're the most "natural" or
;; "common-sense-like" for this task.

;; Define simple truth value
(define (stv mean conf) (cog-new-stv mean conf))

;; Shorthand for the node types
(define VN VariableNode)
(define PN PredicateNode)
(define CN ConceptNode)
(define AN FeatureNode) ; AvatarNode

;; Clause containing relating three things of three types
(define (clause t1 v1 t2 v2 t3 v3)
	(EvaluationLink
		(t1 v1)
		(ListLink
			(t2 v2)
			(t3 v3)
		)
	)
)

;; Declare a variable var to be of type type
(define (decl-var type var)
	(TypedVariableLink
		(VariableNode var)
		(VariableTypeNode type)
	)
)

;; "Is the same person" deduction rule.
;; If person A and person B both share the same predicate and property,
;; then they must be the same person.
(define (is-same-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "PredicateNode" "$predicate")
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "ConceptNode" "$attribute")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... 
			(AndLink
				(clause VN "$predicate" VN "$person_a" VN "$attribute")
				(clause VN "$predicate" VN "$person_b" VN "$attribute")
				;; Avoid reporting things we already know.
				;; Basically, if we already know that person A and B
				;; are the same person, then lets not deduce it again.
				;; This not link is identical to the conclusion below
				(NotLink
					(clause PN "IsSamePerson" VN "$person_a" VN "$person_b")
				)
			)
			;; implicand -- then the following is true too
			(clause PN "IsSamePerson" VN "$person_a" VN "$person_b")
		)
	)
)


;; transitive deduction rule.
;; If property X holds for person A, and person A is same as person B
;; then property X also holds for person B.
(define (transitive-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "PredicateNode" "$predicate")
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "ConceptNode" "$attribute")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause VN "$predicate" VN "$person_a" VN "$attribute")
				(clause PN "IsSamePerson" VN "$person_a" VN "$person_b")
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				(NotLink
					(clause VN "$predicate" VN "$person_b" VN "$attribute")
				)
			)
			;; implicand -- then the following is true too
			(clause VN "$predicate" VN "$person_b" VN "$attribute")
		)
	)
)


;; Houses at the end of the street can only have one neighbor, ever.
;; This is a rather narrow rule, as it can only ever apply to the first
;; address (first ordinal -- a boundary condition).
;; This is used to combine rules 9 and 14.
;; There should be a symmetric rule for the last address too ...
(define (first-addr-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "ConceptNode" "$addr_a")
			(decl-var "ConceptNode" "$addr_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... 
			(AndLink
				;; if adress of personA is 1st house
				(clause PN "Address" VN "$person_a" CN "101 Main Street")
				(clause PN "Address" VN "$person_a" VN "$addr_a")
				;; and A is neighbor of B
				(clause PN "Neighbor" VN "$person_a" VN "$person_b")
				;; and the next house is one over
				(clause PN "Successor" VN "$addr_a" VN "$addr_b")
				;; and we don't already know the conclusion
				(NotLink
					(clause PN "Address" VN "$person_b" VN "$addr_b")
				)
			)
			;; implicand -- then the B lives one house over.
			(clause PN "Address" VN "$person_b" VN "$addr_b")
		)
	)
)



;; neighbor deduction rule.
;; If Address X is left of address Y, then person who lives in X is
;; a neighbor of person who lives in Y
(define (neighbor-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "ConceptNode" "$addr_a")
			(decl-var "ConceptNode" "$addr_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause PN "Address" VN "$person_a" VN "$addr_a")
				(clause PN "Address" VN "$person_b" VN "$addr_b")
				(clause PN "Successor" VN "$addr_a" VN "$addr_b")
				; Not interested in what we already know.
				(NotLink
					(clause PN "Neighbor" VN "$person_a" VN "$person_b")
				)
			)
			;; implicand -- then the following is true too
			(clause PN "Neighbor" VN "$person_a" VN "$person_b")
		)
	)
)

;; neighbor relation is symmetric
;; If A is a neighbor of B then B is a neighbor of A
(define (neighbor-symmetry-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause PN "Neighbor" VN "$person_a" VN "$person_b")
				; Not interested in what we already know.
				(NotLink
					(clause PN "Neighbor" VN "$person_b" VN "$person_a")
				)
			)
			;; implicand -- then the following is true too
			(clause PN "Neighbor" VN "$person_b" VN "$person_a")
		)
	)
)


;; Deduce if a solution has been found ... this simply tries to see
;; if all attributes have been deduced, and if so, just clumps them
;; together.
(define (found-solution-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person")
			(decl-var "ConceptNode" "$nationality")
			(decl-var "ConceptNode" "$drink")
			(decl-var "ConceptNode" "$smoke")
			(decl-var "ConceptNode" "$pet")
			(decl-var "ConceptNode" "$house")
			(decl-var "ConceptNode" "$addr")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause PN "Nationality" VN "$person" VN "$nationality")
				(clause PN "Drinks"      VN "$person" VN "$drink")
				(clause PN "Smokes"      VN "$person" VN "$smoke")
				(clause PN "Keeps"       VN "$person" VN "$pet")
				(clause PN "LivesIn"     VN "$person" VN "$house")
				(clause PN "Address"     VN "$person" VN "$addr")

				;; Don't report a fact we already know.
				(NotLink
         		(OrderedLink
						(VN "$nationality")
						(VN "$drink")
						(VN "$smoke")
						(VN "$pet")
						(VN "$house")
						(VN "$addr")
					)
				)
			)
			;; implicand -- We're just going to use a plain-old ordered
			;; link here to report the results. Why not ...
         (OrderedLink
				(VN "$nationality")
				(VN "$drink")
				(VN "$smoke")
				(VN "$pet")
				(VN "$house")
				(VN "$addr")
			)
		)
	)
)



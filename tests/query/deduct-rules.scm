;;
;; deduct-rules.scm
;;
;; Deduction rules for Einstein puzzle
;;

;; Define simple truth value
(define (stv mean conf) (cog-new-stv mean conf))

;; Declare a variable var to be of type type
(define (decl-var type var)
	(TypedVariableLink
		(VariableNode var)
		(VariableTypeNode type)
	)
)

;; Shorthand for the node types
(define VN VariableNode)
(define PN PredicateNode)
(define CN ConceptNode)
(define AN AvatarNode)

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

;; "Is the same person" deduction rule.
;; If person A and person B both share the same predicate and property,
;; then they must be the same person.
(define (is-same-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "PredicateNode" "$predicate")
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
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
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
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
;; address (first ordinal -- a boundary condition)
;; There should be a symmetric rule for the last address too ...
(define (first-addr-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
			(decl-var "ConceptNode" "$addr_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... 
			(AndLink
				;; if adress of personA is 1st house
				(clause PN "Address" VN "$person_a" CN "101 Main Street")
				;; and A is neighbor of B
				(clause PN "Neighbor" VN "$person_a" VN "$person_b")
				;; and the next house is one over
				(clause PN "Successor" CN "101 Main Street" VN "$addr_b")
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
			(decl-var "AvatarNode" "$person_a")
			(decl-var "AvatarNode" "$person_b")
			(decl-var "ConceptNode" "$house_a")
			(decl-var "ConceptNode" "$house_b")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause PN "Address" VN "$person_a" VN "$house_a")
				(clause PN "Address" VN "$person_b" VN "$house_b")
				(clause PN "Successor" VN "$house_a" VN "$house_b")
			)
			;; implicand -- then the following is true too
			(clause PN "Neighbor" VN "$person_a" VN "$person_b")
		)
	)
)




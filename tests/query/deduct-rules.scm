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

(use-modules (srfi srfi-1))

;; Define simple truth value
(define (stv mean conf) (cog-new-stv mean conf))

;; Shorthand for the node types
(define VN VariableNode)
(define PN PredicateNode)
(define CN ConceptNode)
(define AN FeatureNode) ; AvatarNode

;; Predicate clause specifies a predicate that associates attribute to person
(define (clause t1 v1 t2 v2 t3 v3)
	(EvaluationLink
		(t1 v1)
		(ListLink
			(t2 v2)
			(t3 v3)
		)
	)
)

;; Predicate clause negating the third attribute.
(define (not-clause t1 v1 t2 v2 t3 v3)
	(EvaluationLink
		(t1 v1)
		(ListLink
			(t2 v2)
			(NotLink (t3 v3))
		)
	)
)

;; Predicate clause, asserting that v2 and v3 are different atoms.
(define (differ t2 v2 t3 v3)
	(EvaluationLink
		(GroundedPredicateNode "c++:exclusive")
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

;; ---------------------------------------------------------------------
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

;; ---------------------------------------------------------------------
;; Transitive deduction rule.
;;
;; If attribute X holds for person A, and person A is same as person B
;; then attribute X also holds for person B.
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

;; ---------------------------------------------------------------------
;; Transitive-not deduction rule.
;;
;; If attribute X doesn't hold for person A, and person A is same as person B
;; then attribute X also doesn't hold for person B.
;;
;; Very similar to above
(define (transitive-not-rule)
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
				(not-clause VN "$predicate" VN "$person_a" VN "$attribute")
				(clause PN "IsSamePerson" VN "$person_a" VN "$person_b")
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				(NotLink
					(not-clause VN "$predicate" VN "$person_b" VN "$attribute")
				)
			)
			;; implicand -- then the following is true too
			(not-clause VN "$predicate" VN "$person_b" VN "$attribute")
		)
	)
)

;; ---------------------------------------------------------------------
;; elimination

(define (by-elimination-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person")
			(decl-var "PredicateNode" "$predicate")
			(decl-var "ConceptNode" "$attr_a")
			(decl-var "ConceptNode" "$attr_b")
			(decl-var "ConceptNode" "$attr_c")
			(decl-var "ConceptNode" "$attr_d")
			(decl-var "ConceptNode" "$attr_e")
			(decl-var "ConceptNode" "$attr_type")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				;; If person does NOT have atttribute a,b,c or d ...
				(not-clause VN "$predicate" VN "$person" VN "$attr_a")
				(not-clause VN "$predicate" VN "$person" VN "$attr_b")
				(not-clause VN "$predicate" VN "$person" VN "$attr_c")
				(not-clause VN "$predicate" VN "$person" VN "$attr_d")
				;; and the attributes a,b,c,d,e are all of the same kind
				(InheritanceLink (VN "$attr_a") (VN "$attr_type"))
				(InheritanceLink (VN "$attr_b") (VN "$attr_type"))
				(InheritanceLink (VN "$attr_c") (VN "$attr_type"))
				(InheritanceLink (VN "$attr_d") (VN "$attr_type"))
				(InheritanceLink (VN "$attr_e") (VN "$attr_type"))
				;; and attributes a,b,c,d,e are all different from one-another
				(EvaluationLink
					(GroundedPredicateNode "c++:exclusive")
					(ListLink
						(VN "$attr_a")
						(VN "$attr_b")
						(VN "$attr_c")
						(VN "$attr_d")
						(VN "$attr_e")
					)
				)
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				;(NotLink
				;	(clause VN "$predicate" VN "$person" VN "$attr_e")
				;)
			)

			;; implicand -- then the following is true too
			;; Then by elimination, person must have attribute e.
			(clause VN "$predicate" VN "$person" VN "$attr_e")
		)
	)
)

;; ---------------------------------------------------------------------
;; distinct-attr rule.
;; If, for a given attribute, person a and person b take on different
;; values, then they cannot be the same person.  Therefore, any other
;; attributes they have must also be exclusive.

(define (distinct-attr-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "PredicateNode" "$predicate_common")
			(decl-var "ConceptNode" "$attribute_comm_a")
			(decl-var "ConceptNode" "$attribute_comm_b")
			(decl-var "PredicateNode" "$predicate_exclusive")
			(decl-var "ConceptNode" "$attribute_excl")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause VN "$predicate_common" VN "$person_a" VN "$attribute_comm_a")
				(clause VN "$predicate_common" VN "$person_b" VN "$attribute_comm_b")
				(differ VN "$attribute_comm_a" VN "$attribute_comm_b")
				(clause VN "$predicate_exclusive" VN "$person_a" VN "$attribute_excl")
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				;(NotLink
				;	(not-clause VN "$predicate_exclusive" VN "$person_b" VN "$attribute_excl")
				;)
			)

			;; implicand -- then the following is true too
			(not-clause VN "$predicate_exclusive" VN "$person_b" VN "$attribute_excl")
		)
	)
)

;; ---------------------------------------------------------------------
;; neighbor-not-attr rule.
;; If some attribute holds true for a person, it cannot hold for the
;; person's neighbor.

(define (neighbor-not-attr-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(decl-var "FeatureNode" "$person_a")
			(decl-var "FeatureNode" "$person_b")
			(decl-var "PredicateNode" "$predicate")
			(decl-var "ConceptNode" "$attribute")
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(clause VN "$predicate" VN "$person_a" VN "$attribute")
				(clause PN "Neighbor" VN "$person_a" VN "$person_b")
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				(NotLink
					(not-clause VN "$predicate" VN "$person_b" VN "$attribute")
				)
			)

			;; implicand -- then the following is true too
			(not-clause VN "$predicate" VN "$person_b" VN "$attribute")
		)
	)
)

;; ---------------------------------------------------------------------
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

;; ---------------------------------------------------------------------
;; Neighbor deduction rule.
;;
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

;; ---------------------------------------------------------------------
;; Neighbor relation is symmetric
;;
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

;; ---------------------------------------------------------------------
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



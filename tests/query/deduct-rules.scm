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

;; Clause containing three variables
(define (clause-vvv v1 v2 v3)
	(EvaluationLink
		(VariableNode v1)
		(ListLink
			(VariableNode v2)
			(VariableNode v3)
		)
	)
)

;; Clause containing one predicate and two variables
(define (clause-pvv p1 v2 v3)
	(EvaluationLink
		(PredicateNode p1)
		(ListLink
			(VariableNode v2)
			(VariableNode v3)
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
				(clause-vvv "$predicate" "$person_a" "$attribute")
				(clause-vvv "$predicate" "$person_b" "$attribute")
				;; Avoid reporting things we already know.
				;; Basically, if we already know that person A and B
				;; are the same person, then lets not deduce it again.
				;; This not link is identical to the conclusion below
				(NotLink
					(clause-pvv "IsSamePerson" "$person_a" "$person_b")
				)
			)
			;; implicand -- then the following is true too
			(clause-pvv "IsSamePerson" "$person_a" "$person_b")
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
				(clause-vvv "$predicate" "$person_a" "$attribute")
				(clause-pvv "IsSamePerson" "$person_a" "$person_b")
				;; Don't deduce thigs we already know...
				;; i.e. this not link is identical to conclusion, below.
				(NotLink
					(clause-vvv "$predicate" "$person_b" "$attribute")
				)
			)
			;; implicand -- then the following is true too
			(clause-vvv "$predicate" "$person_b" "$attribute")
		)
	)
)

;; Clause containing one predicate, one variable, one constant
(define (clause-pvc p1 v2 c3)
	(EvaluationLink
		(PredicateNode p1)
		(ListLink
			(VariableNode v2)
			(ConceptNode c3)
		)
	)
)

;; Clause containing one predicate, one variable, one constant
(define (clause-pvc p1 v2 c3)
	(EvaluationLink
		(PredicateNode p1)
		(ListLink
			(VariableNode v2)
			(ConceptNode c3)
		)
	)
)

;; Clause containing one predicate, one variable, one constant
(define (clause-pcv p1 c2 v3)
	(EvaluationLink
		(PredicateNode p1)
		(ListLink
			(ConceptNode c2)
			(VariableNode v3)
		)
	)
)


;; Houses at the end of the street can only have one neighbor, ever.
;; This is a rather narrow rule, used in very narrow circumstances.
(define (first-house-rule)
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
				(clause-pvc "Address" "$person_a" "101 Main Street")
				;; and A is neighbor of B
				(clause-pvv "Neighbor" "$person_a" "$person_b")
				;; and the next house is one over
				(EvaluationLink
					(PredicateNode "Successor")
					(ListLink
						(ConceptNode "101 Main Street")
						(VariableNode "$addr_b")
					)
				)
				;; and we don't already know the conclusion
				(NotLink
					(EvaluationLink
						(PredicateNode "Address")
						(ListLink
							(VariableNode "$person_b")
							(VariableNode "$addr_b")
						)
					)
				)
			)
			;; implicand -- then the B lives one house over.
			(EvaluationLink
				(PredicateNode "Address")
				(ListLink
					(VariableNode "$person_b")
					(VariableNode "$addr_b")
				)
			)
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
				(EvaluationLink
					(PredicateNode "Address")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$house_a")
					)
				)
				(EvaluationLink
					(PredicateNode "Address")
					(ListLink
						(VariableNode "$person_b")
						(VariableNode "$house_b")
					)
				)
				(EvaluationLink
					(PredicateNode "Successor")
					(ListLink
						(VariableNode "$house_a")
						(VariableNode "$house_b")
					)
				)
			)
			;; implicand -- then the following is true too
			(EvaluationLink
				(PredicateNode "Neighbor")
				(ListLink
					(VariableNode "$person_a")
					(VariableNode "$person_b")
				)
			)
		)
	)
)




;;
;; buggy-not.scm
;;
;; Crazy NotLink bug
;;

(define (stv mean conf) (cog-new-stv mean conf))

;; The Englishman lives in the red house.
(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person1")
		(ConceptNode "red_house")
	)
)

;; The person who lives in the red house keeps fish.
(EvaluationLink (stv 1 1)
	(PredicateNode "LivesIn")
	(ListLink
		(AvatarNode "person2")
		(ConceptNode "red_house")
	)
)


;;
;; deduct-rules.scm
;;
;; Deduction rules for Einstein puzzle
;;

;; "Is the same person" deduction rule.
;; If person A and person B both share the same predicate and property,
;; then they must be the same person.
(define (is-same-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(TypedVariableLink
				(VariableNode "$predicate")
				(VariableTypeNode "PredicateNode")
			)
			(TypedVariableLink
				(VariableNode "$person_a")
				(VariableTypeNode "AvatarNode")
			)
			(TypedVariableLink
				(VariableNode "$person_b")
				(VariableTypeNode "AvatarNode")
			)
			(TypedVariableLink
				(VariableNode "$property")
				(VariableTypeNode "ConceptNode")
			)
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(EvaluationLink
					(VariableNode "$predicate")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$property")
					)
				)
				(EvaluationLink
					(VariableNode "$predicate")
					(ListLink
						(VariableNode "$person_b")
						(VariableNode "$property")
					)
				)
				;; Avoid reporting things we already know.
				;; Basically, if we already know that person A and B
				;; are the same person, then lets not deduce it again.
				(NotLink
					(EvaluationLink
						(PredicateNode "IsSamePerson")
						(ListLink
							(VariableNode "$person_a")
							(VariableNode "$person_b")
						)
					)
				)
			)
			;; implicand -- then the following is true too
			(EvaluationLink
				(PredicateNode "IsSamePerson")
				(ListLink
					(VariableNode "$person_a")
					(VariableNode "$person_b")
				)
			)
		)
	)
)


;; transitive deduction rule.
;; If property X holds for person A, and person A is same as person B
;; then property X also holds for person B.
;(define (transitive-rule)
	;(BindLink
	;	;; variable declarations
	;	(ListLink
	;		(TypedVariableLink
	;			(VariableNode "$predicate")
	;			(VariableTypeNode "PredicateNode")
	;		)
	;		(TypedVariableLink
	;			(VariableNode "$person_a")
	;			(VariableTypeNode "AvatarNode")
	;		)
	;		(TypedVariableLink
	;			(VariableNode "$person_b")
	;			(VariableTypeNode "AvatarNode")
	;		)
	;		(TypedVariableLink
	;			(VariableNode "$property")
	;			(VariableTypeNode "ConceptNode")
	;		)
	;	)
	;	(ImplicationLink
	;		;; body -- if all parts of AndLink hold true ... then
	;		(AndLink
	;			(EvaluationLink
	;				(VariableNode "$predicate")
	;				(ListLink
	;					(VariableNode "$person_a")
	;					(VariableNode "$property")
	;				)
	;			)
	;			(EvaluationLink
	;				(PredicateNode "IsSamePerson")
	;				(ListLink
	;					(VariableNode "$person_a")
	;					(VariableNode "$person_b")
	;				)
	;			)
	;		)
	;		;; implicand -- then the following is true too
	;		(EvaluationLink
	;			(VariableNode "$predicate")
	;			(ListLink
	;				(VariableNode "$person_b")
	;				(VariableNode "$property")
	;			)
	;		)
	;	)
	;)
;)


(define (wtf-rule)
	(BindLink
		;; variable declarations
		(ListLink
			(TypedVariableLink
				(VariableNode "$predicate")
				(VariableTypeNode "PredicateNode")
			)
			(TypedVariableLink
				(VariableNode "$person_a")
				(VariableTypeNode "AvatarNode")
			)
			(TypedVariableLink
				(VariableNode "$person_b")
				(VariableTypeNode "AvatarNode")
			)
		)
		(ImplicationLink
			;; body -- if all parts of AndLink hold true ... then
			(AndLink
				(EvaluationLink
					(PredicateNode "IsSamePerson")
					(ListLink
						(VariableNode "$person_a")
						(VariableNode "$person_b")
					)
				)
			)
			;; implicand -- then the following is true too
			(OrderedLink
				(VariableNode "$predicate")
				(VariableNode "$person_a")
				(VariableNode "$person_b")
;				(VariableNode "$property")
			)
		)
	)
)

 (define (prt-atomspace) 
   (define (prt-atom h) 
     ; print only the top-level atoms.
     ; (if (null? (cog-incoming-set h)) 
         (display h) ;)   
   #f)
   (define (prt-type type)
     (cog-map-type prt-atom type)
     ; We have to recurse over sub-types
     (for-each prt-type (cog-get-subtypes type))
   )
   (prt-type 'NotLink)                          
 )




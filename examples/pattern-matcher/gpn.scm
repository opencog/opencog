;
; GroundedPredicateNode Demo
;
; During pattern matching, it can sometimes be useful to call a
; special-purpose function to decide if a given grounding is correct
; or not.  This can be done using the GroundedPredicateNode, as shown
; in this demo.
;
; The run this, you probably need to do this:
;
; OCDIR=home/home/yourname/opencog
; export LTDL_LIBRARY_PATH=$OCDIR/build/opencog/guile:$OCDIR/build/opencog/query
;
; Add the following to your ~/.guile file:
; (add-to-load-path "/home/yourname/opencog/build")
; (add-to-load-path "/home/yourname/opencog/opencog/scm")
; (add-to-load-path ".")
;
; Start guile:
; guile
;
; and then load this file:
; (load-from-path "markov-chain.scm")
;
; Then, scroll to the bottom, and some of the commented-out
; examples.
;
(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

; Define a function that takes an atom and returns an OpenCog truth
; value. In this case, it randomly returns true or false, about half the
; time.  The function can take zero, two or more arguments, but it must
; always return an opencog truth value, as the truth value will be used
; during pattern matching to make a grounding decision.
(define (rand-ok atom)
	(define r (random 2))
	(simple-format #t "Random number: ~A " r)
	(if (< 0 r)
		(begin
			(simple-format #t "Picked ~A\n" (cog-name atom))
			(stv 1 1) ; return true
		)
		(begin
			(simple-format #t "Did not pick ~A\n" (cog-name atom))
			(stv 0 1) ; return false
		)
	)
)

; The function can be invoked directly, using the cog-evaluate!
; function. The below defines an EvaluationLink that, when evaluated,
; sometimes picks something, and sometimes doesn't.
;
(define sometimes
	(EvaluationLink
		(GroundedPredicateNode "scm: rand-ok")
		(ListLink
			(ConceptNode "something"))))

; Try it!  run the following a few times:
; (cog-evaluate! sometimes)

; The pattern-matching requires some data in the atomspace to match
; against. So populate the atomspace with some data.
(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Aristotle")
		(ConceptNode "logician")
	)
)

(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "CS Pierce")
		(ConceptNode "logician")
	)
)

;; The followig pattern will search for all logicians in the AtomSpace,
;; and then will randomly select some of them, with a 50-50 chance each
;; time. The propsoed grounding, made in the first clause of the
;; pattern, is randomly approved of or rejected by the second clause.
(define find-logicians
	(BindLink
		; Dfine the variable to be grounded
		(VariableNode "$person")
		(ImplicationLink
			; Define a list of two clauses, both of which must be satsified
			(AndLink
				; The first clause: find a grounding for the variable, such
				; that the variable is grounded by the name of a logician.
				(EvaluationLink
					(PredicateNode "is-a")
					(ListLink
						(VariableNode "$person")
						(ConceptNode "logician")
					)
				)
				; The second clause: the propsed grounding, from above,
				; is randomly accepted or rejected.  Several of these can be
				; combined using AndLink, OrLink and NotLink.
				(EvaluationLink
					(GroundedPredicateNode "scm: rand-ok")
					(ListLink
						(VariableNode "$person")
					)
				)
			)
			; Return the grounding, if selected.
			(VariableNode "$person")
		)
	)
)

; Running the below multiple times will return different sets of
; selected logicians each time.
;
; (cog-bind find-logicians)

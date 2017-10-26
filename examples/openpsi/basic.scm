;
; Basic example of OpenPsi usage
;
;
; Psi rules have the general form
;
;     ImplicationLink  <TV>
;        SequentialAndLink
;           <context>
;           <action>
;        <goal>
;
; The intended meaning of these rules is that if the context is
; applicable, and the action is taken, then the goal is fulfilled.
;
; The context is meant to be one or more EvaluationLinks (or boolean
; combinations thereof).  If these evaluate to true, then the action
; will be taken.  The action must also be of the form of a predicate,
; (that is, myst be an EvaluationLink, or a boolean combination
; thereof).  If the action also evaluates to true, then the goal is
; considered to be fulfilled.

(use-modules (opencog)  (opencog openpsi))

(define (check-situation atom)
	(display "Checking the current situation: ")
	(display atom)
	(stv 1 1)
)

;; Return true only if the result of performing the action
;; actually fulfilled the goal.
(define (action-doit atom)
	(display "Taking the action: ")
	(display atom)
	(stv 1 1)
)

(define (goal-update atom)
	(display "Updating the goal for: ")
	(display atom)
	(stv 0.1 1)
)

(define demand-foo (psi-demand "foo"))
(define demand-bar (psi-demand "bar"))

(Member
	(psi-rule
		(list
			(Evaluation (GroundedPredicate "scm: check-situation")
				(List (Concept "foo"))))
		(Evaluation (GroundedPredicate "scm: action-doit")
				(List (Concept "foo-action")))
		(Evaluation (GroundedPredicate "scm: goal-update")
				(List (Concept "foo-goal")))
		(stv 0.9 1)
		demand-foo
	)
	(Concept "demo rule")
)

(Member
	(psi-rule
		(list
			(Evaluation (GroundedPredicate "scm: check-situation")
				(List (Concept "bar"))))
		(Evaluation (GroundedPredicate "scm: action-doit")
				(List (Concept "bar-action")))
		(Evaluation (GroundedPredicate "scm: goal-update")
				(List (Concept "bar-goal")))
		(stv 0.9 1)
		demand-bar
	)
	(Concept "demo rule")
)


; Single-step the psi rule-selection engine `n` times.
; This will print a message, single-step the psi rule engine, and
; then sleep for three seconds.  The loop repeats until the loop count
; reaches zero.
(define (step-psi n)
	(display "\n=================== Stepping psi one step\n")
	(psi-step-per-demand)
	(sleep 3)
	(if (< 0 n) (step-psi (- n 1))))

(step-psi 4)
(display "Done running psi")

; Alternately, one can run the psi engine as fast as possible. This
; is not recommended for this demo, because it will clog the output
; with print statements.
; (psi-run-per-demand)

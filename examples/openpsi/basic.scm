;
; Basic example of OpenPsi usage
;

(use-modules (opencog)  (opencog openpsi))

(define (check-situation atom)
	(display "Checking the current situation: ")
	(display atom)
	(stv 1 1)
)

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

(define demand-foo (psi-demand "foo" 0.8))
(define demand-bar (psi-demand "bar" 0.5))

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


; Single-step the psi rule-selection engine.
; This will print a message, single-step the psi rule engine, and
; then sleep for three seconds.  The loop repeats until the loop count
; reaches zero.
(define (step-psi n)
	(display "\n=================== Stepping psi one step\n")
	(psi-step)
	(sleep 3)
	(if (< 0 n) (step-psi (- n 1))))

(step-psi 4)
(display "Done running psi")

; Alternately, one can run the psi engine as fast as possible. This
; is not recommended for this demo, because it will clog the output
; with print statements.
; (psi-run)


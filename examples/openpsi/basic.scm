;
; Basic example of OpenPsi usage
;

(use-modules (opencog)  (opencog openpsi))

(define (situation atom)
	(display "Hello situation\n")
	(stv 1 1)
)

(define (doit atom)
	(display "going to do it\n")
	(stv 1 1)
)

(define (demand-it atom)
	(display "Hello demand\n")
	(stv 1 1)
)

(define demand-foo (psi-demand "foo" 0.8))
(define demand-bar (psi-demand "bar" 0.5))

(Member
	(psi-rule
		(list
			(Evaluation (GroundedPredicate "scm: situation")
				(List (Concept "foobar"))))
		(Evaluation (GroundedPredicate "scm: doit")
				(List (Concept "barfoo")))
		(Evaluation (GroundedPredicate "scm: demand-it")
				(List (Concept "demndoid")))
		(stv 0.9 1)
		demand-foo
	)
	(Concept "demo rule")
)


; Single-step the psi rule-selection engine.
; This will print a message, single-step the psi rule engine, and
; then sleep for one second.  The loop repeats until the loop count
; reaches zero.
(define (step-psi n)
	(display "\n=================== Stepping psi one step\n")
	(psi-step)
	(sleep 1)
	(if (< 0 n) (step-psi (- n 1))))

(step-psi 4)
(display "Done running psi")

; Alternately, one can run the psi engine as fast as possible. This
; is not recommended for this demo, because it will clog the output
; with print statements.
; (psi-run)


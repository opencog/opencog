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
	(Concept "done it")
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
		(ExecutionOutput (GroundedSchema "scm: doit")
				(List (Concept "barfoo")))
		(Evaluation (GroundedPredicate "scm: demand-it")
				(List (Concept "demndoid")))
		(stv 0.9 1)
		demand-foo
	)
	(Concept "demo rule")
)


(psi-run)


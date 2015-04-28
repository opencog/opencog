;
; GroundedShemaNode Demo.
;
; After a pattern-match has been found, arbitrary code can be
; triggered to run.  This demo shows how.
;
; Running arbitary functions upon match can be useful for any number
; of reasons:  to send a message whenever a match is found; to perform
; some particularly complex or odious computation, and so on.
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
; (load-from-path "gsn.scm")
;
; Then, scroll to the bottom, and some of the commented-out
; examples.

(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

;; Some arbitrary function, taking one atom as an argument.
;; This function could take zero, two or more arguments; however,
;; in general, it should always return an atom.  It doesn't have to;
;; if it returns something else, then that will e discarded and
;; replaced by the invalid atom.
(define (say-hello atom)
	(display "Hello, ")
	(display (cog-name atom))
	(display "!")
	(newline)
	atom
)

;; Executing the below will cause the "say-hello" function to be called,
;; with the list of atoms in the ListLink being the arguments to the
;; function. To execute, do this:
;;
;;    (cog-execute! say-hello-to-linas)
;;
;; The number of atoms in the ListLink should correspond to the number
;; of arguments of the called function. Failure for the two to
;; correspond will generally result in a hang or crash.
(define say-hello-to-linas
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: say-hello")
			(ListLink (ConceptNode "Linas"))))


;; ExecutionOutputLinks are particularly useful when combined with the
;; pattern matcher. They can be used to invoke a function every time
;; that a match is found.
;;
;; To demonstrate this, we need to populate the atomspace with some data.
;; In this case, some assertions about who is human.
(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Linas")
		(ConceptNode "human")
	)
)

(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Ben")
		(ConceptNode "human")
	)
)

;; Define a pattern that searches for everyone who is a human, and then
;; invokes the say-hello function on each match.
(define find-humans
	(BindLink
		;; Declare the variable to be grounded.
		(VariableNode "$person")
		(ImplicationLink
			;; The pattern to be searched for.
			(EvaluationLink
		   	(PredicateNode "is-a")
   			(ListLink
					(VariableNode "$person")
      			(ConceptNode "human")
   			)
			)
			;; The proceedure to invoke when a grounding is found.
			(ExecutionOutputLink
		   	(GroundedSchemaNode "scm: say-hello")
   			(ListLink
					(VariableNode "$person")
   			)
			)
		)
	)
)

;; The below should cause two hello messages to be printed, when
;; it is run.
;;
; (cog-bind find-humans)

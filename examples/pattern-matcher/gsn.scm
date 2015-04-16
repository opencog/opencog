
(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

;; Some arbitrary function, taking one atom as an argument. 
;; This function could take zero, two or more arguments; however,
;; in general, it should always return an atom.  It doesn't have to;
;; if it returns something else, then that will e discarded and
;; replaced by the invalid atom.
(define (say-hello atom)
	(display "hello!")
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

;; For the next part, we need to populate the atomspace with some data.
;; In this case, some assertions about who is human.
(EvaluationLink
	(PredicateNode "is-a")
	(ListLink
		(ConceptNode "Linas")
		(ConceptNode "human")
	)
)


(define find-humans
	(BindLink
		(VariableNode "$person")
		(ImplicationLink
			(EvaluationLink
		   	(PredicateNode "is-a")
   			(ListLink
					(VariableNode "$person")
      			(ConceptNode "human")
   			)
			)
			(EvaluationLink
		   	(GroundedSchemaNode "scm: say-hello")
   			(ListLink
					(VariableNode "$person")
   			)
			)
		)
	)
)

; (cog-bind find-humans)

(cog-execute!  (ExecutionOutputLink
      (GroundedSchemaNode "scm: say-hello")
      (ListLink
         (ConceptNode "Linas")
      )
   )
)
(define (say-hello atom) (display "hello!") (newline) atom)
(define (say-hello atom too) (display "hello!") (newline) too)


(cog-execute!  (ExecutionOutputLink (GroundedSchemaNode "scm: say-hello") (ListLink (ConceptNode "Linas") (ConceptNode "joe"))))


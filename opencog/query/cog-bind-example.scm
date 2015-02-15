;
; implication-example.scm
;
; Linas Vepstas January 2009
;
; This file contains a simple, visual-inspection demo example for the 
; use of the query engine to find pattern matches using the scheme
; interfaces.  The below defines a single BindLink and two chunks of
; data, and then calls the pattern matcher.
;
; This example can be run in two different ways.  The first way requires
; that the cogserver be running. Then either cut-n-paste, or cat the file:
;    cat cog-bind-example.scm | telnet localhost 17001
;
; To see the results, telnet to either port 17001 or 18001
;    telnet localhost 17001
; and then run the final step:
;    (cog-bind x)
;
; The second way of running this is to run it without the cogserver,
; using only guile:
;
;    export LTDL_LIBRARY_PATH=build/opencog/guile:build/opencog/query
;    cd ../..
;    cat opencog/query/cog-bind-example.scm | guile -L build -L opencog/scm
;
; The expected result, after running the below, is that the following
; should be printed:
;
; guile> (cog-bind x)
; (ListLink (EvaluationLink (PredicateNode "make_from")
;     (ListLink (ConceptNode "pottery")
;            (ConceptNode "clay"))))
;
; Note the outermost ListLink is simply an enumeration of all of the 
; possible results from the implication; in this case, there is only one
; possible result.
;

(use-modules (opencog))
(use-modules (opencog query))
(load-from-path "utilities.scm")

(define x
	(BindLink
		(ListLink
			(VariableNode "$var0")
			(VariableNode "$var1")
			(VariableNode "$verb")
		)
		(ImplicationLink 
			(AndLink
				(EvaluationLink
					(PredicateNode "_obj")
					(ListLink
						(VariableNode "$verb") ; (ConceptNode "make")
						(VariableNode "$var0") ; (ConceptNode "pottery")
					)
				)
				(EvaluationLink
					(PredicateNode "from")
					(ListLink
						(VariableNode "$verb") ; (ConceptNode "make")
						(VariableNode "$var1") ; (ConceptNode "clay")
					)
				)
			)
			(EvaluationLink
				(PredicateNode "make_from")
				(ListLink
					(VariableNode "$var0")
					(VariableNode "$var1")
				)
			)
		)
	)
)

(define o
	(EvaluationLink (stv 1.0 1.0)
		(PredicateNode "_obj")
		(ListLink
			(ConceptNode "make")
			(ConceptNode "pottery")
		)
	)
)
(define f
	(EvaluationLink  (stv 1.0 1.0)
		(PredicateNode "from")
		(ListLink
			(ConceptNode "make")
			(ConceptNode "clay")
		)
	)
)

(cog-bind x)

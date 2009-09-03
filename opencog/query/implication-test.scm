scm
;
; implication-test.scm
;
; Linas Vepstas January 2009
;
; This file contains a simple, visual-inspection test for the operation
; of the query engine (in opencog/query) in the handling of
; ImplicationLinks. The below defines a single implication link and
; two chunks of data, and then calls the implication engine. 
;
; The expected result, after running the below, is that the following
; should be printed:
;
; guile> (cog-ad-hoc "do-implication" x)
; (ListLink (EvaluationLink (PredicateNode "make_from")
;     (ListLink (ConceptNode "pottery")
;            (ConceptNode "clay"))))
;
; Note the outermost ListLink is simply an enumeration of all of the 
; possible results from the implication; in this case, there is only one
; possible result.
;
; Note that this test has been largely superceeded by the automated unit
; tests for the query engine. But its still useful here as a demo of 
; "what's going on".
;
(define x
	(ImplicationLink 
		(AndLink
			(EvaluationLink
				(PredicateNode "_obj")
				(ListLink
					(ConceptNode "make")
					(VariableNode "$var0")
				)
			)
			(EvaluationLink
				(PredicateNode "from")
				(ListLink
					(ConceptNode "make")
					(VariableNode "$var1")
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

(cog-ad-hoc "do-implication" x)

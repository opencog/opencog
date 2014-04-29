
(define (dbg)

	(define olde *-atomspace-*)
	(let ((*-atomspace-* (cog-new-atomspace))
			)
			(list olde *-atomspace-*)
	)
)

(define (func-env)

	; This is very cheesy, but it does work ... 
	; Create a new atomspce, set the current environment atomspace
	; pointer to it, create an atom, then set it back to the old
	; value.  What we really want to do, someday, is to have the
	; smob code fetch the atomspace from the current environment.
	(define olde *-atomspace-*)
	(let ((newe (cog-new-atomspace))
			(h2 #f)
			)
		(set! *-atomspace-* newe)
		(set! h2 (ConceptNode "stuff"))
		(set! *-atomspace-* olde)
		h2
	)
)

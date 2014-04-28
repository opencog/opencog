
(define (dbg)

	(define olde *-atomspace-*)
	(let ((*-atomspace-* (cog-new-atomspace)))
		(list olde *-atomspace-*)
	)
)

(define (func-env)

	; This is sort-of cheesy, but it does work ... 
	; Create a new atomspace, set the current environment atomspace
	; pointer to it, create an atom, then set it back to the old
	; value.  
	(let* ((olde *-atomspace-*)
			(newe (cog-new-atomspace))
			(h2 #f)
			)
		(set! *-atomspace-* newe)
		(set! h2 (ConceptNode "stuff"))
		(set! *-atomspace-* olde)
		h2
	)
)

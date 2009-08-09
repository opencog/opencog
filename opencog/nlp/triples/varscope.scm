;
; varscope.scm
;
; Wrapper to create proper VariableScopeLinks from the naked
; ImplicationLinks that the perl files generate.
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
; varscope-wrap-inplication implication
;
; Create a VariableScopeLink wrapping the ImplicationLink
; This just simply locates all of the VariableNodes in the
; ImplicationLink, and declares them up-front in the VariableScopeLink.
;
(define (varscope-wrap-inplication impl)

	; Find all VariableNode's in the implication,
	; and return them as a list.
	(define (find-vars atoms)
		(define (is-var? atom)
			(eq? 'VariableNode (cog-type atom))
		)
		(filter-hypergraph is-var? atoms)
	)

	; Create a VariableScopeLink declaring all of the 
	; VariableNodes that we found.
	(let* ((all-vars (find-vars impl))
			(vars (delete-duplicates! all-vars))
		)
		(VariableScopeLink
			(ListLink
				vars
			)
			impl
		)
	) 
)

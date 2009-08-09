;
; varscope.scm
;
; Wrapper to create proper VariableScopeLinks from the naked
; ImplicationLinks that the perl files generate.
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
;
; Create a VarScopeLink wrapping the ImplicationLink
;
(define (varscope-wrap impl)

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
	(let ((vars (find-vars impl)))
		(VariableScopeLink
			(ListLink
				vars
			)
			impl
		)
	) 
)

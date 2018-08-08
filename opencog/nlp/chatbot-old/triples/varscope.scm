;
; varscope.scm
;
; Wrapper to create proper BindLinks from the naked
; ImplicationLinks that the perl files generate.
;
; XXX This is not used any more, because the perl script is not used any
; more.  This is currently kept here because it might still come in 
; handy during the conversion of the RelEx frame rules into pure
; opencog.  Delete this file when frames have been removed from RelEx
; (or fully added to OpenCog) XXX
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
; varscope-wrap-inplication implication
;
; Create a BindLink wrapping the ImplicationLink
; This just simply locates all of the VariableNodes in the
; ImplicationLink, and declares them up-front in the BindLink.
;
(define (varscope-wrap-implication impl)

	; Find all VariableNode's in the implication,
	; and return them as a list.
	(define (find-vars atoms)
		(define (is-var? atom)
			(eq? 'VariableNode (cog-type atom))
		)
		(filter-hypergraph is-var? atoms)
	)

	; Create a BindLink declaring all of the 
	; VariableNodes that we found.
	(let* ((all-vars (find-vars impl))
			(vars (delete-duplicates! all-vars))
		)
		(BindLink
			(VariableList
				vars
			)
			impl
		)
	)
)

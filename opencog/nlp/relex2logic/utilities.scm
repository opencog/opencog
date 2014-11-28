;
; utilities.scm
;
; Assorted utilties for checking R2L outputs.
;

; -----------------------------------------------------------------------
; r2l-get-root -- Return all hypergraph R2L roots containing 'atom'
;
; Similar to cog-get-root, except that it will stop at the SetLink in case
; RelEx2Logic is called with the r2l(...) function.
;
(define (r2l-get-root atom)
	(define iset (cog-incoming-set atom))
	
	; if reached the SetLink that wrap around R2L outputs (happens when using r2l(...))
	(if (and (= (length iset) 1) (equal? 'SetLink (cog-type (car iset))))
		(list atom)
		; if no incoming set (happens when using relex-parse(...))
		(if (null? iset)
			(list atom)
			(append-map r2l-get-root iset)
		)
	)
)

; -----------------------------------------------------------------------
; r2l-get-word-inst -- Retrieve WordInstanceNode given R2L style node
;
; Given a R2L style node, find the corresponding WordInstanceNode.
;
(define (r2l-get-word-inst node)
	(cond ((null? node) '())
	      ((has-word-inst? node) (cog-node 'WordInstanceNode (cog-name node)))
	      (else '())
	)
)

; -----------------------------------------------------------------------
; has-word-inst? -- Check if a node has the corresponding WordInstanceNode
;
; Return #t or #f depends on whether the node as a WordInstanceNode.
;
(define (has-word-inst? node)
	(not (null? (cog-node 'WordInstanceNode (cog-name node))))
)

; -----------------------------------------------------------------------
; has-word? -- Check if a node has the corresponding WordNode
;
; Return #t or #f depends on whether the node as a WordNode.
;
(define (has-word? node)
	(not (null? (cog-node 'WordNode (cog-name node))))
)

; -----------------------------------------------------------------------
; is-r2l-inst? -- Check if a node is a R2L instanced node.
;
; Return #t or #f depends on whether the node is instanced.  VariableNode
; is also instanced node.
;
(define (is-r2l-inst? node)
	(or (equal? 'VariableNode (cog-type node)) (has-word-inst? node))
)

; -----------------------------------------------------------------------
; is-r2l-abstract? -- Check if a node is a R2L abstract node.
;
; Return #t or #f depends on whether the node is the abstract version.
;
(define (is-r2l-abstract? node)
	(has-word? node)
)


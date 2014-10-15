
; =======================================================================
; Helper functions
; =======================================================================

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
; word-inst-has-attr? -- Check if a WordInstanceNode has a particular attribute
;
; Return #t or #f depends on whether the node has the attribute 'attr-string' or not.
;
(define (word-inst-has-attr? word-inst attr-string)
	(define attr-list (word-inst-get-attr word-inst))
	(and (not (null? attr-list)) (member attr-string attr-list (lambda (s a) (string=? s (cog-name a)))))
)

; -----------------------------------------------------------------------
; cog-has-node? -- Check if "atom" contains node "target"
;
; Return #t or #f depends on whether the hypergraph "atom" contains the
; node "target.  Basically doing BFS.
;
(define (cog-has-node? atom target)
	(define (recursive-helper queue)
		(cond ((null? queue)
			#f
		      )
		      ((equal? (car queue) target)
			#t
		      )
		      (else
			(if (cog-link? (car queue))
				(set! queue (append queue (cog-outgoing-set (car queue))))
			)
			(recursive-helper (cdr queue))
		      )
		)
	)

	(recursive-helper (append (list atom) (cog-outgoing-set atom)))
)

; -----------------------------------------------------------------------
; and-l -- Apply 'and' operator to a list
;
; Helper function for since we cannot do (apply and ...)
;
(define (and-l x)
	(if (null? x)
		#t
		(if (car x) (and-l (cdr x)) #f)
	)
)

; -----------------------------------------------------------------------
; sublist -- Returns a sublist of a list.
;
; Returns a sublist from index 'start' (inclusive) to index 'end' (exclusive)
;
(define (sublist l start end)
	(define len (length l))
	(if (or (>= start len) (< end start))
		'()
		(take (drop l start) (- end start))
	)
)

; -----------------------------------------------------------------------
; form-graph-match? -- Basic pattern match between "atom" and "form"
;
; Check "atom" against a sentence form "form" to see if the two structures
; are the same, and if specified, also check if POS matches.
;
(define (form-graph-match? atom form)
	(if (cog-node? atom)
		(if (equal? (cog-type atom) (cog-type form))
			; check if we need to check POS or not
			(if (string=? (cog-name form) "_")
				#t
				(word-inst-match-pos? (r2l-get-word-inst atom) (cog-name form))
			)
			#f
		)
		(if (and (= (cog-arity atom) (cog-arity form)) (equal? (cog-type atom) (cog-type form)))
			; check the outgoing set recursively
			(and-l (map form-graph-match? (cog-outgoing-set atom) (cog-outgoing-set form)))
			#f
		)
	)
)

; -----------------------------------------------------------------------
; match-sentence-forms -- Helper function to see if an atom matches a sentence form
;
; Check "atom" against a list of sentence forms and returns the first form
; matched;  returns #f if none matched.
;
(define (match-sentence-forms atom favored-forms)
	(find (lambda (form) (form-graph-match? atom form)) favored-forms)
)

; -----------------------------------------------------------------------
; is-object? -- Check if 'node' is "(indirect) object" within 'link'
;
; Given a link 'link', check if the node 'node' can be considered "object"
; or "indirect object" as in "subject-verb-object".
;
(define (is-object? link node)
	; subject-object property must be in EvaluationLink
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		; (indirect) object must be in a ListLink with the subject
		(if (and (equal? 'ListLink (cog-type (gdr link))) (> (cog-arity (gdr link)) 1))
			; check either "object" or "indirect object"
			(if (= (cog-arity (gdr link)) 2)
				(equal? node (gddr link))
				(equal? node (caddr (cog-outgoing-set (gdr link))))
			)
			#f
		)
	)
)

; -----------------------------------------------------------------------
; get-subject -- Get the subject of a link
;
; Given a link 'link', get the node that can be considered "subject" as
; in "subject-verb-object".
;
(define (get-subject link)
	; similar to is-object? check, must be in EvaluationLink
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		(if (equal? 'ListLink (cog-type (gdr link)))
			(gadr link)
			(gdr link)
		)
	)
)

; -----------------------------------------------------------------------
; distance-transform -- 2D distance transform
;
; Not used at the moment.
;
(define (distance-transform main-list anchor-list)
	; initialize where the anchors are
	; XXX optimization possible? This makes the whole function O(mn) rather than O(n)
	(define len (length main-list))
	(define result-list (map (lambda (i) (if (member i anchor-list) 0 1)) main-list))

	(define (left2right-helper sub-list curr-dist)
		(cond ((null? sub-list)
			'()
		      )
		      ((= (car sub-list) 0)
			(cons 0 (left2right-helper (cdr sub-list) 1))
		      )
		      (else
			(cons curr-dist (left2right-helper (cdr sub-list) (+ 1 curr-dist)))
		      )
		)
	)

	(define (right2left-helper sub-list curr-dist)
		(cond ((null? sub-list)
			'()
		      )
		      ((= (car sub-list) 0)
			(cons 0 (right2left-helper (cdr sub-list) 1))
		      )
		      (else
			(if (< (car sub-list) curr-dist)
				(cons (car sub-list) (right2left-helper (cdr sub-list) (+ 1 curr-dist)))
				(cons curr-dist (right2left-helper (cdr sub-list) (+ 1 curr-dist)))
			)
		      )
		)				
	)

	(set! result-list (left2right-helper result-list len))
	(set! result-list (reverse result-list))
	(set! result-list (right2left-helper result-list len))
	(set! result-list (reverse result-list))
	result-list
)


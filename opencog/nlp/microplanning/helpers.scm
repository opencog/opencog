
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

(define (word-inst-has-attr? word-inst attr-string)
	(define attr-list (word-inst-get-attr word-inst))
	(and (not (null? attr-list)) (member attr-string attr-list (lambda (s a) (string=? s (cog-name a)))))
)

(define (word-inst-is-singular? word-inst)
	(word-inst-has-attr word-inst "singular")
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


; helper function to see if an atom matches a sentence form
(define (match-sentence-forms atom favored-forms)
	(find (lambda (form) (form-graph-match? atom form)) favored-forms)
)


(define (is-subject? link node)
	; a node can only be subject if within an EvaluationLink
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		(if (equal? 'ListLink (cog-type (gdr link)))
			(equal? node (gadr link))
			(equal? node (gdr link))
		)
	)
)

(define (is-object? link node)
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		(if (and (equal? 'ListLink (cog-type (gdr link))) (> (cog-arity (gdr link)) 1))
			(equal? node (gddr link))
			#f
		)
	)
)

(define (get-subject link)
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		(if (equal? 'ListLink (cog-type (gdr link)))
			(gadr link)
			(gdr link)
		)
	)
)

; not used at the moment
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

; -----------------------------------------------------------------------
; check-chunk -- Check if a chunk is "say-able"
;
; Call Surface Realization pipeline to see if a chunk of atoms are
; "say-able", and if so, determines whether the sentence is too long or
; complex.
;
; Returns 0 = not sayable, 1 = sayable, 2 = too long/complex
;
(define (check-chunk atoms)
	(define (get-pos n)
		(define wi (r2l-get-word-inst n))
		(if (null? wi)
			"_"
			(cog-name (car (word-inst-get-pos wi)))
		)
	)
	(define (sort-n-count l)
		(define sorted-list (sort l string<?))
		(define (count-helper curr-subset curr-pos curr-count)
			(cond ((null? curr-subset)
				(cons (cons curr-pos curr-count) '())
			      )
			      ((not (string=? (car curr-subset) curr-pos))
				(cons (cons curr-pos curr-count) (count-helper (cdr curr-subset) (car curr-subset) 1))
			      )
			      (else
				(count-helper (cdr curr-subset) curr-pos (+ 1 curr-count))
			      )
			)
		)
		(if (not (null? sorted-list))
			(count-helper (cdr sorted-list) (car sorted-list) 1)
			'()
		)
	)

	; XXX optimization needed?
	(define all-nodes (delete-duplicates (append-map cog-get-all-nodes atoms)))
	(define pos-alist (sort-n-count (map get-pos all-nodes)))
	
	(define (pos-checker al)
		(define pos (car al))
		(define count (cdr al))
		
		(cond ; prefer max 2 verbs per sentence
		      ((and (string=? "verb" pos) (>= count 2))
			#f
		      )
		      ; prefer max 3 nouns per sentence
		      ((and (string=? "noun" pos) (>= count 3))
			#f
		      )
		      ; prefer max 5 adjectives
		      ((and (string=? "adj" pos) (>= count 5))
			#f
		      )
		      ; prefer max 3 adverbs
		      ((and (string=? "adv" pos) (>= count 3))
			#f
		      )
		      (else
			#t
		      )
		 )
	)

	(define pos-result (and-l (map pos-checker pos-alist)))

	;;; do something with SuReal to see if it is sayable?
	(define say-able #t)

	(cond 
	      ; not long/complex but sayable
	      ((and pos-result say-able) 1)
	      ; long/complex but sayable
	      ((and (not pos-result) say-able) 2)
	      ; not sayable
	      (else 0)
	)

	;(length (append-map cog-get-all-nodes atoms))
)


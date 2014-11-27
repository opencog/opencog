
; =======================================================================
; Helper functions
; =======================================================================

; -----------------------------------------------------------------------
; set-values! -- Helpful macro for storing multiple return values
;
; The macro allows binding of multiple return values to several variables,
; without having to define a subscope like (receive).
;
; Can be used like:
;
;	(define a)
;	(define b)
;	(set-values! (a b) (values 5 7))
;
; resulting a = 5, b = 7
;
(define-syntax set-values!
	(syntax-rules ()
		((_ () exp)
			(values)
		)
		((_ (v1 . rest) exp)
			(begin
				(set! v1 (call-with-values (lambda () exp) list))
				(set-values! rest (apply values (cdr v1)))
				(set! v1 (car v1))
			)
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
; cog-has-atom-type? -- Check if "atom" contains atom type "type"
;
; Return #t or #f depends on whether the hypergraph "atom" contains an
; atom of type "type".  Basically doing BFS.
;
(define (cog-has-atom-type? atom type)
	(define (recursive-helper queue)
		(cond ((null? queue)
			#f
		      )
		      ((equal? (cog-type (car queue)) type)
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
	; if the form is the wildcard node, don't care what it matches to
	(if (equal? form (VariableNode "MicroplanningWildcardMarker"))
		#t
		(if (cog-node? atom)
			(if (equal? (cog-type atom) (cog-type form))
				; check if we need to check POS or not
				(if (string=? (cog-name form) "MicroplanningAnyNameMarker")
					#t
					(let ((pos (cond ((equal? (cog-name form) "MicroplanningVerbMarker")
							  "verb"
							 )
							 (else
							  "unknown"
							 )
						   )
					     ))
					     (word-inst-match-pos? (r2l-get-word-inst atom) pos)
					)
				)
				#f
			)
			(if (and (= (cog-arity atom) (cog-arity form)) (equal? (cog-type atom) (cog-type form)))
				; check the outgoing set recursively
				(every form-graph-match? (cog-outgoing-set atom) (cog-outgoing-set form))
				#f
			)
		)
	)
)

; -----------------------------------------------------------------------
; match-sentence-forms -- Helper function to see if an atom matches a sentence form
;
; Check "atom" and its subgraph against a list of sentence forms and returns
; the subgraph that got matched;  returns #f if none matched a sentence form.
;
; Also returns the base atom index (in the order returned from cog-get-all-nodes)
; where the first node in the matched subgraph would start.  The atom index
; is useful for
;
;    (EvaluationLink (PredicateNode "punched") (ListLink (ConceptNode "I") (ConceptNode "I"))
;
; where the same (ConceptNode "I") might be changed differently bases on position.
;
(define (match-sentence-forms atom favored-forms)
	(define atom-base-index 0)
	(define (helper-start atom form)
		(set! atom-base-index 0)
		(helper atom form)
	)

	; helper function to allow subgraph matching
	(define (helper atom form)
		(if (form-graph-match? atom form)
			atom
			; check if subgraph match the form
			(if (cog-link? atom)
				(any (lambda (subgraph) (helper subgraph form)) (cog-outgoing-set atom))
				(begin
					(set! atom-base-index (+ atom-base-index 1))
					#f
				)
			)
		)
	)

	(values (any (lambda (form) (helper-start atom form)) favored-forms) atom-base-index)
)

; -----------------------------------------------------------------------
; cog-pred-is-argN? -- Check if 'node' is argument N within 'link'
;
; Given an EvaluationLink 'link', check if the node 'node' is argument N
; (starts from 0).
;
(define (cog-pred-is-argN? link node N)
	; must be in EvaluationLink, and N >= 0
	(if (or (< N 0) (not (equal? 'EvaluationLink (cog-type link))))
		#f
		(if (equal? 'ListLink (cog-type (gdr link)))
			(if (> (cog-arity (gdr link)) N)
				(equal? node (list-ref (cog-outgoing-set (gdr link)) N))
				#f
			)
			(if (= N 0)
				(equal? node (gdr link))
				#f
			)
		)
	)
)

; -----------------------------------------------------------------------
; cog-pred-get-argN -- Get argument number N of an EvaluationLink
;
; Given a EvaluationLink 'link', get argument N.  The returned node can
; be considered "subject" as in "subject-verb-object".
;
(define (cog-pred-get-argN link N)
	; must be in EvaluationLink, and N >= 0
	(if (or (< N 0) (not (equal? 'EvaluationLink (cog-type link))))
		#f
		(if (equal? 'ListLink (cog-type (gdr link)))
			(if (> (cog-arity (gdr link)) N)
				(list-ref (cog-outgoing-set (gdr link)) N)
				#f
			)
			(if (= N 0)
				(gdr link)
				#f
			)
		)
	)
)

; -----------------------------------------------------------------------
; cog-pred-get-pred -- Get the predicate of an EvaluationLink
;
; Get the PredicateNode of a specific EvaluationLink 'link'.
;
(define (cog-pred-get-pred link)
	; must be in EvaluationLink
	(if (not (equal? 'EvaluationLink (cog-type link)))
		#f
		(gar link)
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


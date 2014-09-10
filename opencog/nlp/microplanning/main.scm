(load "sentence-forms.scm")


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
	      ((word-instance? node) (cog-node 'WordInstanceNode (cog-name node)))
	      (else '())
	)
)

; -----------------------------------------------------------------------
; word-instance? -- Check if a node has the corresponding WordInstanceNode
;
; Return #t or #f depends on whether the node as a WordInstanceNode.
;
(define (word-instance? node)
	(not (null? (cog-node 'WordInstanceNode (cog-name node))))
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
; pos-match? -- Check if a R2L "node" has part-of-speech "target-pos"
;
; Return #t for "node" if the corresponding WordInstanceNode has
; PartOfSpeechLink to type "target-pos" (eg. verb, noun, adj).
;
(define (pos-match? node target-pos)
	(define word-inst (r2l-get-word-inst node))

	(if (null? word-inst)
		#f
		(string=? (cog-name (car (word-inst-get-pos word-inst))) target-pos)
	)
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
				(pos-match? atom (cog-name form))
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



; =======================================================================
; Main microplanning functions
; =======================================================================

; -----------------------------------------------------------------------
; microplanning -- The main microplanning function call
;
; Accepts a SequentialAndLink containing a list of atoms to be spoken.
; "utterance-type" is either 'declarative', 'interrogative', 'imperative'
; or 'interjective'
;
(define (microplanning seq-link utterance-type)
	; initialize the sentence forms as needed
	(microplanning-init)

	; should utterance-type be part of the link (or just outside) instead of passing as parameter
	(make-utterance (cog-outgoing-set seq-link) utterance-type)
)

; -----------------------------------------------------------------------
; make-utterance -- The main helper function for handling a microplanning request
;
; Calls make-sentence repeatedly until all informations have been spoken, or
; no longer say-able.
;
(define (make-utterance atoms-set utterance-type)
	(define atoms-unused atoms-set)
	(define sentence-chunks '())
	(define (recursive-helper)
		(define new-chunk '())

		; keep calling make sentence until all atoms are used
		(cond ((not (null? atoms-unused))
			(set! new-chunk (make-sentence atoms-unused atoms-set utterance-type))
			(set! sentence-chunks (cons new-chunk sentence-chunks))
			; XXX keep some of the atoms (those that do not satisfy sentence forms) for later use?
			(set! atoms-unused (lset-difference equal? atoms-unused new-chunk))
			(recursive-helper)
		      )
		)
	)
	(define (call-sureal atoms)
		; wrap a SetLink around the atoms
		(sureal (SetLink atoms))
	)

	; loop make-sentence on remaining atoms
	(recursive-helper)

	(reverse sentence-chunks)

	; call SuReal on each item in sentence-chunks
	;(map call-sureal (reverse sentence-chunks))
)

; -----------------------------------------------------------------------
; make-sentence -- Create a single sentence
;
; Greedily add new atoms (using some heristics) to form a new sentence.
;
(define (make-sentence atoms-unused atoms-set utterance-type)
	; find the preferred sentence forms base on utterance-type
	(define favored-forms
		(cog-outgoing-set (car
			(cond ((string=? "declarative" utterance-type)
				(cog-chase-link 'InheritanceLink 'OrLink (ConceptNode "declarative"))
			      )
			      ((string=? "interrogative" utterance-type)
				'()
			      )
			      ((string=? "imperative" utterance-type)
				'()
			      )
			      ((string=? "interjective" utterance-type)
				'()
			      )
			)
		))
	)

	; helper function to see if an atom matches a sentence form
	(define (match-sentence-forms atom)
		(find (lambda (form) (form-graph-match? atom form)) favored-forms)
	)

	; initialize weight bases on "time ordering"
	(define time-weights '())
	; initialize weight (0, 1) bases on whether the atom satisfy a sentence form
	(define form-weights '())
	; initialize weight bases on linkage to other atoms
	(define link-weights '())

	; comb-proc should takes in param "time", "form", "link" in this order
	(define (calc-weights choices bases comb-proc)
		(define choices-len (length choices))

		; helper function: get all nodes for a atom, get all nodes in bases, count how many common nodes
		(define (link-count atom)
			(define atom-nodes (cog-get-all-nodes atom))
			; XXX need to optimize performance here?
			(define used-nodes (delete-duplicates (append-map cog-get-all-nodes bases)))
			(length (lset-intersection equal? atom-nodes used-nodes))
		)

		(set! time-weights (list-tabulate choices-len (lambda (i) (- choices-len i))))
		(set! form-weights (map (lambda (atom) (if (match-sentence-forms atom) 1 0)) choices))
		(set! link-weights (map link-count choices))
		
		(display "current weights\n")
		(display time-weights)
		(display "\n")
		(display form-weights)
		(display "\n")
		(display link-weights)
		(display "\nweights done\n")

		; create a combined weight using comb-proc
		(map comb-proc time-weights form-weights link-weights)
	)

	(define atoms-used (lset-difference equal? atoms-set atoms-unused))
	(define atoms-failed '())
	(define chunk '())

	; helper function to sort the atoms list base on weights and choose the highest one
	(define (pick-atom a-list weights)
		(define assoc-list (sort (map cons a-list weights) (lambda (x y) (> (cdr x) (cdr y)))))
		(caar assoc-list)
	)

	; helper function for looping
	(define (recursive-helper atoms-to-try)
		(define head (car atoms-to-try))
		(define result (check-chunk atoms-to-try))

		(define (give-up-unadded-part)
			; atoms that did not work
			(define dead-set (lset-difference atoms-to-try chunk))
			; atoms that worked
			(define good-set (lset-intersection atoms-to-try chunk))

			(set! atoms-failed (append dead-set atoms-failed))
			(set! atoms-unused (lset-difference dead-set atoms-unused))

			; try "saying" the previous working iteration again and re-choose a new atom
			(recursive-helper good-set)
		)
		(define (update-chunk)
			; add anything that has not been included in the chunk yet
			(set! chunk (lset-union equal? atoms-to-try chunk))
			(set! atoms-unused (lset-difference equal? atoms-unused atoms-to-try))
			(display "updating chunk\n")
			(display chunk)
			(display "\nupdating chunk ended\n")
		)

		; temporary variables not all conditions needed
		(define temp-var1 '())
		(define temp-var2 '())
		(define temp-differences '())

		(display "trying atoms\n")
		(display atoms-to-try)
		(display "\n....\n")

		(cond ; not say-able
		      ((= result 0)
			(display "not say-able\n")
			; could be the atoms cannot be said unless bringing in additional atoms (such as "and/or/that")
			; try to add more (up to 3 different links)
			(cond ((<= (length atoms-to-try) 3)
				; look to see if the newest link has any ConceptNode that is solo
				(set! temp-var1 (cog-get-all-nodes head))
				(set! temp-var2 (append-map cog-get-all-nodes (cdr atoms-to-try)))
				(set! temp-differences (filter word-instance? (lset-difference temp-var1 temp-var2)))

				(cond ((null? temp-differences)
					(give-up-unadded-part)
				      )
				      (else
					; find the first link in atoms-unused that contains one of the solo word
					(set! temp-var1
						(find (lambda (a)
							(find (lambda (w) (cog-has-node? a w)) temp-differences)
						     )
						     atoms-unused
						)
					)

					; if an atom with the solo word exists
					(if (temp-var1)
						(recursive-helper (cons temp-var1 atoms-to-try))
						(give-up-unadded-part)
					)
				      )
				)
			      )
			      ; give up on all the atoms not "say-able"
			      (else
				(give-up-unadded-part)
			      )
			)
		      )
		      ; not long/complex enough
		      ((= result 1)
			(display "result not long enough\n")
			; add the currently "say-able" stuff to our chunk
			(update-chunk)

			; only continue if there are more to say
			(if (not (null? atoms-unused))
				(recursive-helper
					(cons
						; weight is (time-weights + link-weights) * (2 - form-weights), ie. prefer links that do not
						; satisify a sentence form, meaning it cannot be "said" on its own.
						(pick-atom atoms-unused (calc-weights atoms-unused chunk (lambda (t f l) (* (+ t l) (- 2 f)))))
						atoms-to-try
					)
				)
			)
		      )
		      ; too long/complex
		      (else
			(display "result too long\n")
			(update-chunk)
		      )
		)
	)

	; the initial critera for choosing a starting point would be (time-weights + link-weights) * form-weights
	(recursive-helper (list (pick-atom atoms-unused (calc-weights atoms-unused atoms-used (lambda (t f l) (* (+ t l) f))))))

	; return the sentence chunk (reverse because we've been adding to the front)
	(reverse chunk)
)


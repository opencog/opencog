;
;
; Loading additional dependencies
; Attention: we need to `include`, not `load` helpers.scm because it
; contains a macro definition. This macro causes unit-test failures,
; ... Hmm ... because the file compile step is broken? is this due to
; a guile bug??  Anyway, for guile-2.1, taken from git, as of March
; 2016, saying `load` here fails.  Its tied to the set-values! macro.
;
(include-from-path "opencog/nlp/microplanning/helpers.scm")
(load-from-path "opencog/nlp/microplanning/anaphora-nouns-list.scm")

; =======================================================================
; Main anaphora insertion functions
; =======================================================================

; -----------------------------------------------------------------------
; insert-anaphora -- The main function for inserting anaphora
;
; Accepts a <chunks-set>.  Returns new <chunks-set> with anaphora inserted
; (as new atoms).
;
; TODO also insert anaphora for missing subjects/objects
;
(define (insert-anaphora inputs-set)
	(define results-set (clone-set inputs-set))
	(define changed-chunk-indicator (make-list (get-length inputs-set) #f))
	(define n-lst (make <nouns-list>))
	
	(populate-nouns-list n-lst (get-chunks inputs-set))

	; go through each noun-item in the noun-list
	(for-each-by-proc n-lst
		(lambda (ni)
			(let* ((forms (get-sentence-forms (get-utterance-type inputs-set (get-chunk-index ni))))
			       (old-noun-node (get-noun-node ni))
			       (new-noun-node
				; if the noun can be a pronoun, get a copy of the new node with the pronoun as name
				(if (is-pronoun-safe? ni)
					; unless we are indicating the possession link can be deleted
					(if (equal? (get-modified-pronomial ni forms) 'possessed-link-cancel)
						'possessed-link-cancel
						(cog-new-node (cog-type old-noun-node) (get-modified-pronomial ni forms) (cog-tv old-noun-node))
					)
					; otherwise, try getting a node with the lexical noun
					; XXX possibly better algorithm for choosing between pronoun vs lexical noun phrase?
					(if (is-lexical-safe? ni)
					 	(if (get-association n-lst ni)
			 				(get-lexical-node (get-association n-lst ni))
			 				(get-lexical-node ni)
			 			)
			 			; cannot be replaced? return null to indicate this
			 			'()
			 		)
			 	)))
			 	; if the old noun usage can be replaced
			 	(if (not (nil? new-noun-node))
			 		(begin
			 			; indicate the chunk is changed and need checking with SuReal
			 			(list-set! changed-chunk-indicator (get-chunk-index ni) #t)
						(if (equal? new-noun-node 'possessed-link-cancel)
							; replace the possession link with '() to delete it
							(mod-link results-set (get-chunk-index ni) (get-link-index ni) '())
							(let* (; using the link from results-set instead of (get-orig-link ni) because this
							       ; link could have been modified already, and we will to keep the changes
							       (old-link (get-link results-set (get-chunk-index ni) (get-link-index ni)))
							       (new-link (clone-link-and-replace-node old-link (get-atom-index ni) new-noun-node))
							      )
								; do not want a copy of the link for every changes to the node, just want one
								(if (not (equal? (get-orig-link ni) old-link))
									(cog-extract! old-link)
								)
							      
								(mod-link results-set (get-chunk-index ni) (get-link-index ni) new-link)
							)
					 	)
					 )
				 )
			 )	
		)
	)

	; Try to SuReal changed chunk again.  If any chunk failed, return the original chunk without the anaphora.
	(for-each 
		(lambda (changed index)
			(if changed
				(let* ((chunk (get-chunk results-set index))
				       (ut (get-utterance-type results-set index))
				       (temp-set-link (SetLink (get-utterance-link ut chunk) chunk)))
					; failed to SuReal? bring back the old chunk
					(if (nil? (sureal temp-set-link))
						(mod-chunk results-set index (get-chunk inputs-set index))
					)
				
					(cog-extract! temp-set-link)
				)
			)
		)
		changed-chunk-indicator
		(iota (get-length results-set))
	)

	; return a new chunks-set with anaphora added
	results-set
)


; -----------------------------------------------------------------------
; clone-link-and-replace-node -- Helper function to clone a link and replace a node
;
; Note that we search by a node's index instead of the node itself, because
; a link can have multiple of the same node and we want to replace only one
; instance of them.
;
(define (clone-link-and-replace-node orig-link node-index new-node)
	(define curr-atom-index 0)
	(define (clone-helper sublink)
		(define old-oset (cog-outgoing-set sublink))
		(define (change-old atom)
			(cond ((cog-link? atom)
				(clone-helper atom)
			      )
			      ; found the node at the index we want
			      ((= curr-atom-index node-index)
				(set! curr-atom-index (+ 1 curr-atom-index))
				new-node
			      )
			      (else
				(set! curr-atom-index (+ 1 curr-atom-index))
				atom
			      )
			)
		)
		(define new-oset (map-in-order change-old old-oset))

		(apply cog-new-link (append (list (cog-type sublink) (cog-tv sublink)) new-oset))
	)
	
	(clone-helper orig-link)
)


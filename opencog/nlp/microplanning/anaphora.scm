; loading additional dependency
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")
(load-scm-from-file "../opencog/nlp/microplanning/anaphora-nouns-list.scm")

; =======================================================================
; Main anaphora insertion functions
; =======================================================================

; -----------------------------------------------------------------------
; insert-anaphora -- The main function for inserting anaphora
;
; Accepts a list of 'chunks' from make-sentence-chunks and the matching
; list of sentence forms from sentences-form.scm (eg. the list from
; "declarative").  Returns new chunks with anaphora inserted (as
; new atoms).
;
(define (insert-anaphora chunks forms-list)
	; helper function to clone at 'link' level, creating a copy of the link with
	; some of the nouns replaced with pronouns
	(define (clone-link-with-pronoun n-lst orig-link chunk-index link-index forms)
		(define subset (get-chunk-link-sublist n-lst chunk-index link-index))
		(define curr-atom-index 0)

		(define (clone-helper sublink)
			(define old-oset (cog-outgoing-set sublink))
			(define (change-old atom)
				(define result-ni)
				
				(cond ((cog-link? atom)
					(clone-helper atom)
				      )
				      (else
				        ; find the noun-item with matching atom-index, advance it for next step
				        (set! result-ni (find-by-proc subset (lambda (ni) (= (get-atom-index ni) curr-atom-index))))
					(set! curr-atom-index (+ 1 curr-atom-index))
					
				      	; check if the noun should be replace with pronoun
				      	(cond ((and result-ni (is-pronoun-safe? result-ni))
				      		; XXX do we need this new node to inherit from original?
				      		; XXX do it like anaphora resolution?
				      		(cog-new-node (cog-type atom)
			      				(get-modified-pronomial result-ni forms)
			      				(cog-tv atom))
				      	      )
				      	      (else
				      		atom
				      	      )				      	
				      	)
				      )
				)
			)
			(define new-oset (map-in-order change-old old-oset))
			
			(apply cog-new-link (append (list (cog-type sublink) (cog-tv sublink)) new-oset))
		)
		
		; if subset empty, return original link (ie. the link has no noun)
		(if (null? subset)
			orig-link
			; clone, and if found a node within ns and ds is true, create a new node with new name ps
			(clone-helper orig-link)
		)	
	)
	
	; helper function to clone at 'chunk' level
	(define (clone-chunk-with-pronoun n-lst chunk chunk-index forms)
		(map clone-link-with-pronoun (circular-list n-lst) chunk (circular-list chunk-index) (iota (length chunk)) (circular-list forms))
	)
	
	(define n-lst (make <nouns-list>))
	
	(populate-nouns-list n-lst chunks)
	(map clone-chunk-with-pronoun (circular-list n-lst) chunks (iota (length chunks)) forms-list)

	; TODO also insert anaphora for missing subjects/objects, nominal anaphora
)


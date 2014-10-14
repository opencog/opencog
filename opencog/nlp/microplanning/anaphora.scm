; loading additional dependency
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")


; =======================================================================
; Main anaphora insertion functions
; =======================================================================

; -----------------------------------------------------------------------
; insert-anaphora -- The main function for inserting anaphora
;
; Accepts a list of 'chunks' from make-sentence-chunks and the current
; list of sentence forms from sentences-form.scm (eg. the list from
; "declarative").  Returns new chunks with anaphora inserted (as
; new atoms).
;
(define (insert-anaphora chunks favored-forms)
	; XXX checking which atom satisfy a sentence form again... is there ways
	; to store the result from previous check in make-sentence?
	(define form-check-list
		(map (lambda (c) (map (lambda (atom) (if (match-sentence-forms atom favored-forms) #t #f)) c)) chunks)
	)
	
	; helper function for 'nouns-list'
	(define (get-noun atom formed chunk-index link-index)
		(filter-map
			(lambda (x) (if (word-inst-is-noun? (r2l-get-word-inst x))
					(list x formed link-index chunk-index)
					#f))
			(cog-get-all-nodes atom))
	)
	
	; create a noun list of the form:
	;    (noun1 formed1 link-index1 chunk-index1) (noun2 formed2 link-index2 chunk-index2) ...
	; where
	;    noun# is one of the noun as OpenCog node
	;    formed# is #t or #f indicating whether the link satisfy sentence form
	;    link-index# is the link index within a chunk
	;    chunk-index# is the index of the chunk in 'chunks'
	(define nouns-list
		(append-map
			(lambda (c fl ci)
				(append-map (lambda (a f li) (get-noun a f ci li))
					c
					fl
					(iota (length c)) ; generate the links index list
				)
			)
			chunks
			form-check-list
			(iota (length chunks)) ; generate the chunks index list
		)
	)
	
	; bunch of accessors of an item in the nouns-list
	(define (nouns-list-get-noun noun-item)	(car noun-item))
	(define (nouns-list-get-formed noun-item) (cadr noun-item))
	(define (nouns-list-get-li noun-item) (caddr noun-item))
	(define (nouns-list-get-ci noun-item) (cadddr noun-item))
	
	; helper function for pronouns-list that, given a nouns-list item, determine
	; a suitable pronoun
	(define (determine-pronoun n)
		(define word-inst (r2l-get-word-inst (nouns-list-get-noun n)))
		(define is-pronoun (word-inst-has-attr? word-inst "pronoun"))
		(define is-singular (word-inst-has-attr? word-inst "singular"))
		(define is-human (word-inst-has-attr? word-inst "person"))
		(define is-male (and is-human (word-inst-has-attr? word-inst "masculine")))
		
		; TODO check possession for "our", "his", "their", etc. and "ours", "hers", "theirs" etc.
		;    "our group" -> "we"
		;    "our cars" -> "they"
				
		(cond ; do nothing if already pronoun (such as "I", "you", "we")
		      (is-pronoun
			(word-inst-get-word-str word-inst)
		      )
		      ((and is-human is-male)
			"he"
		      )
		      ((and is-human (not is-male))
			"she"
		      )
		      ((and (not is-human) is-singular)
			"it"
		      )
		      ((and (not is-singular))
			"they"
		      )
		)
	)
	
	; generate the base pronoun for each noun
	(define pronouns-list
		(map determine-pronoun nouns-list)
	)
	
	; helper function that, given an index to the nouns-list, check if the corresponding
	; item actually can be changed to pronoun
	(define (check-pronoun index)
		(define the-noun (list-ref nouns-list index))       ; the item in the nouns-list
		(define the-pronoun (list-ref pronouns-list index)) ; the corresponding pronoun
		
		; indicate whether this noun is in a sentence-formed link or not
		(define is-not-well-formed (not (nouns-list-get-formed the-noun)))
		
		; the main helper function to check all conditions
		(define (check-helper)
			; indices of all occurrences of a noun instance in structure ((i1 n1) (i2 n2) ...)
			; excluding occurrences within links that are not sentence-formed
			(define all-occurrences 
				(filter-map 
					(lambda (n i) 
						(if (and (equal? (nouns-list-get-noun n) (nouns-list-get-noun the-noun))
							 (nouns-list-get-formed n)) ; exclude support links
							(cons i n)
							#f))
					nouns-list
					(iota (length nouns-list)) ; generate indices into nouns-list
				)
			)
				
			; check if a prounoun is close to the same pronoun (so would be ambiguous)
			(define (check-ambiguity)
				(define min-index (max 0 (- index 3))) ; inclusive
				(define max-index (min (length pronouns-list) (+ index 3))) ; exclusive
				(define nouns-subset (sublist nouns-list min-index max-index))
				(define pronouns-subset (sublist pronouns-list min-index max-index))
			
				; check how many pronouns in subset equals the-pronoun (except itself)
				(> (count
					(lambda (n p) (and (nouns-list-get-formed n) ; excluce support link
							   (not (equal? (nouns-list-get-noun n) (nouns-list-get-noun the-noun)))
							   (string=? p the-pronoun))) 
					nouns-subset
					pronouns-subset)
				   0
				)
			
				; TODO sometimes it is OK depends on the main subject (current and previous sentence)
				; (eg.  John helped Sam to prepare his project.)
				; (eg.  John helped Sam to feed himself.)
			)
			
			; check if a noun is modified by other non-sentence form links in current chunk,
			; since noun that get supported from supporting link cannot be pronoun
			; (eg. the green it, the tall he, etc)
			(define (check-modified)
				(define chunk-number (nouns-list-get-ci the-noun))
				(find (lambda (n) 
					(if (and (equal? (nouns-list-get-noun n) (nouns-list-get-noun the-noun))
						 (not (nouns-list-get-formed n))
						 (= (nouns-list-get-ci n) (nouns-list-get-ci the-noun)))
						n
						#f))
				      nouns-list
				)
			)
			
			; check if mentioned in last sentence or not
			(define (get-last-mentioned)
				(define this-time (list-index (lambda (o) (= index (car o))) all-occurrences))
				(define last-time (- this-time 1))
			
				(nouns-list-get-ci (cdr (list-ref all-occurrences last-time)))
			)
			
			; gather all the different conditions
			; first check the occurrence list to see if this noun is the first occurrence
			(define is-first-occurrence (= index (caar all-occurrences)))
			; second check if ambiguous
			(define is-ambiguous (check-ambiguity))
			; third check if modified by non-sentenced formed link
			(define is-modified (check-modified))
			; fourth check if mentioned too far back
			(define is-too-far-back
				(and (not is-first-occurrence) (not (>= (+ 1 (get-last-mentioned)) (nouns-list-get-ci the-noun))))
			)
			
			(not (or is-first-occurrence is-ambiguous is-modified is-too-far-back))
		)
		
		; only check if the link is sentence-formed (aka. well formed)
		(if is-not-well-formed
			#f
			(check-helper)
		)
	)
	
	; generate #t & #f bases on whether a noun can be replaced
	(define decisions-list (list-tabulate (length nouns-list) check-pronoun))
	;(define decisions-list (list #t #f #f #f #f #f #t #t))
	
	; helper function to clone at 'link' level, creating a copy of the link with
	; some of the nouns replaced with pronouns
	(define (clone-link-with-pronoun orig-link ns ps ds)
		(define (finalize-pronoun atom pronoun)
			(cond ((not (is-object? orig-link atom))
				pronoun
			      )
			      ; if (indirect) object same as subject
			      ((equal? (get-subject orig-link) atom)
			      	(cond ((string-ci=? "I" pronoun) "myself")
			      	      ((string-ci=? "you" pronoun) "yourself")
			      	      ((string-ci=? "he" pronoun) "himself")
			      	      ((string-ci=? "she" pronoun) "herself")
			      	      ((string-ci=? "it" pronoun) "itself")
			      	      ((string-ci=? "we" pronoun) "ourselves")
			      	      ((string-ci=? "they" pronoun) "themselves")
			      	)
			      )
			      ; if (indirect) object different from subject
			      (else
				(cond ((string-ci=? "I" pronoun) "me")
				      ((string-ci=? "you" pronoun) "you")
				      ((string-ci=? "he" pronoun) "him")
				      ((string-ci=? "she" pronoun) "her")
				      ((string-ci=? "it" pronoun) "it")
				      ((string-ci=? "we" pronoun) "us")
				      ((string-ci=? "they" pronoun) "them")
				)
			      )
			)

		)
	
		(define (clone-helper sublink)
			(define n-index 0)
			(define old-oset (cog-outgoing-set sublink))
			(define (change-old atom)
				(cond ((cog-link? atom)
					(clone-helper atom)
				      )
				      (else
				      	; see if this node is in ns
				      	(set! n-index (list-index (lambda (n) (equal? atom (nouns-list-get-noun n))) ns))
				      	
				      	(cond ((and n-index (list-ref ds n-index))
				      		; XXX do we need this new node to inherit from original?
				      		; XXX do it like anaphore resolution?
				      		(cog-new-node (cog-type atom)
			      				(finalize-pronoun atom (list-ref ps n-index))
			      				(cog-tv atom))
				      	      )
				      	      (else
				      		atom
				      	      )				      	
				      	)
				      )
				)
			)
			(define new-oset (map change-old old-oset))
			
			(apply cog-new-link (append (list (cog-type sublink) (cog-tv sublink)) new-oset))
		)
	
		; if ns empty, return original link
		(if (null? ns)
			orig-link
			; clone, and if found a node within ns and ds is true, create a new node with new name ps
			(clone-helper orig-link)
		)
	)
	
	; helper function to clone at 'chunk' level
	(define (clone-chunk-with-pronoun orig-chunk nouns-subset pronouns-subset decisions-subset)
		(define max-length (length orig-chunk))
		
		(define (recursive-helper link-index)
			(define start-index 0)
			(define end-index 0)
			(cond ((>= link-index max-length)
				'()
			      )
			      (else
				; find subsubset of nouns-subset that share the same link number
				(set! start-index (list-index (lambda (n) (= link-index (nouns-list-get-li n))) nouns-subset))
				(set! end-index (list-index (lambda (n) (< link-index (nouns-list-get-li n))) nouns-subset))

				; if we have reached the end of the subset
				(if (not end-index)
					(set! end-index (length nouns-subset))
				)
				
				; makes it so that we will pass in the previous set (from a link that satisfied sentence form)
				; if this link is not included in nouns-subset
				; TODO find a better way to do this since a supporting link could be for a link sentence form
				; link several index back
				(cond ((not start-index)
					(set! start-index (nouns-list-get-li (list-ref nouns-subset (- end-index 1))))
					(set! start-index (list-index (lambda (n) (= start-index (nouns-list-get-li n))) nouns-subset))
				      )
				)
			
				(cons (clone-link-with-pronoun (list-ref orig-chunk link-index)
							       (sublist nouns-subset start-index end-index)
							       (sublist pronouns-subset start-index end-index)
							       (sublist decisions-subset start-index end-index))
				      (recursive-helper (+ 1 link-index))
				)
			      )
			)
		)
		
		; get back a list of cloned link mixed with original
		(recursive-helper 0)
	)
	
	; the base cloning function
	(define (clone-with-pronoun)
		(define max-length (length chunks))
		
		(define (recursive-helper chunk-index)
			(define start-index 0)
			(define end-index 0)
			
			(cond ((>= chunk-index max-length)
				'()
			      )
			      (else
				; find subset of nouns-list that share the same chunk number
				(set! start-index (list-index (lambda (n) (= chunk-index (nouns-list-get-ci n))) nouns-list))
				(set! end-index (list-index (lambda (n) (< chunk-index (nouns-list-get-ci n))) nouns-list))
				
				; makes it so that we will pass an empty subset if a chunk does not need to be changed
				(if (not start-index)
					(set! start-index (length nouns-list))
				)
				; if we have reached the end of the subset
				(if (not end-index)
					(set! end-index (length nouns-list))
				)
			
				(cons (clone-chunk-with-pronoun (list-ref chunks chunk-index)
								(sublist nouns-list start-index end-index)
								(sublist pronouns-list start-index end-index)
								(sublist decisions-list start-index end-index))
				      (recursive-helper (+ 1 chunk-index))
				)
			      )
			)
		)
		
		(recursive-helper 0)
	)

	(define new-chunks (clone-with-pronoun))


	(display "inserting anaphora...\n")
	(display form-check-list)
	(display "\n")
	(display nouns-list)
	(display "\n")
	(display pronouns-list)
	(display "\n")
	(display decisions-list)
	(display "\n===============\n")
	(display chunks)
	(display "\n===============\n")
	(display "\n...\n")

	; TODO also insert anaphora for missing subjects/objects
	
	new-chunks
)


; loading additional dependency
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")


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
	; XXX checking which atom satisfy a sentence form again... is there ways
	; to store the result from previous check in make-sentence?
	(define form-check-list
		(map (lambda (c fs) (map (lambda (atom) (if (match-sentence-forms atom fs) #t #f)) c)) chunks forms-list)
	)
	
	; ***************************************************************
	; nouns-list code
	;
	; Go through the 'chunks' list and create a nouns list of the form:
	;    (list
	;       (noun1 formed1 link-index1 chunk-index1)
	;       (noun2 formed2 link-index2 chunk-index2)
	;       ...
	;    )
	; where
	;    noun#		the OpenCog node that contains a noun
	;    formed#		#t or #f indicating whether the link satisfy sentence form
	;    link-index#	the link index within a chunk
	;    chunk-index#	the index of the chunk in 'chunks'
	; ***************************************************************
	; helper function for 'nouns-list'
	(define (get-noun atom formed chunk-index link-index)
		(filter-map
			(lambda (node)
				(if (word-inst-is-noun? (r2l-get-word-inst node))
					(list node formed link-index chunk-index) ; forming one item for the noun-list
					#f
				)
			)
			(cog-get-all-nodes atom)
		)
	)
	
	; generate the main nouns-list
	(define nouns-list
		(append-map
			(lambda (c fl chunk-index)
				(append-map
					(lambda (atom formed link-index)
						(get-noun atom formed chunk-index link-index)
					)
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
	
	
	; ***************************************************************
	; pronouns-list code
	;
	; Generate a corresponding list for nouns-list where each noun
	; is examined and determine what would the the appropriate
	; base pronoun be.  Base pronouns are he, she, it, etc.
	; ***************************************************************
	; the main helper function, taking a noun-item as input
	(define (determine-pronoun n)
		(define word-inst (r2l-get-word-inst (nouns-list-get-noun n)))
		(define is-pronoun (word-inst-has-attr? word-inst "pronoun"))
		(define is-singular (word-inst-has-attr? word-inst "singular"))
		(define is-human (word-inst-has-attr? word-inst "person"))
		(define is-male (and is-human (word-inst-has-attr? word-inst "masculine")))
		
		; TODO check possession for "our", "his", "their", etc. and "ours", "hers", "theirs" etc.
		;    "our group" -> "we"
		;    "our cars" -> "they"
		; XXX "our cars" would likely be two seperate nodes in atomspace "we" and "car", possibly
		;     in EvaluationLink "possession", so instead of replacing "our car", we would be 
		;     replacing "car" with pronoun, and delete the "possession" link...
				
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


	; ***************************************************************
	; decisions-list code
	;
	; Actually check each noun to see if it can actually be replaced
	; with a pronoun by looking at the noun's surrounding links,
	; storing either #t or #f as a result.
	; ***************************************************************
	; first stage helper function, accept an index to a noun-item, and
	; check each sentence-formed link
	(define (check-pronoun-stage-1 index)
		(define the-noun (list-ref nouns-list index))       ; the item in the nouns-list
		(define the-pronoun (list-ref pronouns-list index)) ; the corresponding pronoun
		
		; the main helper function to check all conditions
		(define (check-helper)
			; indices of all occurrences of the same noun in structure ((i1 n1) (i2 n2) ...) within
			; 'chunks', excluding occurrences within links that are not sentence-formed.  i1, i2...
			; are indices into nouns-list
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
			;
			; however, need to handle support link for things like about(care, computer) to about(care, it)
			; so an hacky exception with EvaluationLink is added
			;
			; XXX is a better solution possible?
			(define (check-modified)
				(define chunk-number (nouns-list-get-ci the-noun))
				(find (lambda (n) 
					(and (= (nouns-list-get-ci n) chunk-number) ; in the same chunk?
					     (not (nouns-list-get-formed n)) ; found a support link?
					     (equal? (nouns-list-get-noun n) (nouns-list-get-noun the-noun)) ; with the same node?
					     (not (equal? 'EvaluationLink (cog-type (list-ref (list-ref chunks chunk-number) (nouns-list-get-li n))))) ; not an EvaluationLink
					)
				      )
				      nouns-list
				)
			)
			
			; check if mentioned within 2 to 3 sentences back or not
			(define (get-last-mentioned)
				(define this-time (list-index (lambda (o) (= index (car o))) all-occurrences))
				
				; helper function to only get noun in sentence-formed link
				(define (get-last-time ind)
					(if (< ind 0)
						-1
						(if (nouns-list-get-formed (cdr (list-ref all-occurrences ind)))
							ind
							(get-last-time (- ind 1))
						)
					)
				)
				
				(define last-time (get-last-time (- this-time 1)))
			
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
				(and (not is-first-occurrence) (< (+ 3 (get-last-mentioned)) (nouns-list-get-ci the-noun)))
			)
			
			(not (or is-first-occurrence is-ambiguous is-modified is-too-far-back))
		)
	
		; only handle sentence-form link in this stage
		(if (not (nouns-list-get-formed the-noun))
			#f
			(check-helper)
		)
	)

	; generate #t & #f bases on whether a noun can be replaced
	; (calling check-pronoun-stage-1 on each index of the nouns-list)
	(define decisions-list-alpha (list-tabulate (length nouns-list) check-pronoun-stage-1))

	; second stage helper, recheck EvaluationLink's that are not sentence-formed
	; TODO A noun can appear multiple times within one sentence, with some that can be 
	;      changed to pronouns and some that cannot.  Need to figure out which type of
	;      noun the non-sentence-formed link is modifying...
	(define (check-pronoun-stage-2 noun-item prev-decision)	
		(define (check-modifying)
			(any (lambda (n d) 
				(and (= (nouns-list-get-ci n) (nouns-list-get-ci noun-item)) ; in the same chunk?
				     (equal? (nouns-list-get-noun n) (nouns-list-get-noun noun-item)) ; found the same node?
				     d
				)
			     )
			     nouns-list
			     decisions-list-alpha
			)
		)
				
		(if prev-decision
			#t
			(if (and
				(not (nouns-list-get-formed noun-item))
				(equal? 'EvaluationLink (cog-type (list-ref (list-ref chunks (nouns-list-get-ci noun-item)) (nouns-list-get-li noun-item))))
			    )
			    (check-modifying)
			    #f
			)
		)
	)
	
	; check if any non-sentence-fromed link contains nodes that should be changed to
	; pronouns, and modify accordingly
	(define decisions-list (map check-pronoun-stage-2 nouns-list decisions-list-alpha))


	; ***************************************************************
	; cloning code
	;
	; Clone and change nodes with their pronouns as necessary.
	; ***************************************************************
	; helper function to clone at 'link' level, creating a copy of the link with
	; some of the nouns replaced with pronouns
	(define (clone-link-with-pronoun orig-link ns ps ds)
		(define (is-object? lk at)
			(or (cog-pred-is-argN? lk at 1) (cog-pred-is-argN? lk at 2))
		)
		
		(define (finalize-pronoun atom pronoun eval-link)
			; check subject, object, etc, bases on the closest EvaluationLink (so it
			; is possible to match with the subgraph
			(cond ((or (null? eval-link) (not (is-object? eval-link atom)))
				pronoun
			      )
			      ; TODO still need to handle "mine", "hers", "theirs", etc.
			      ; check possession
			      ((and (cog-pred-get-pred eval-link) (string=? (cog-name (cog-pred-get-pred eval-link)) "possession"))
			      	(cond ((string-ci=? "I" pronoun) "my")
			      	      ((string-ci=? "you" pronoun) "your")
			      	      ((string-ci=? "he" pronoun) "his")
			      	      ((string-ci=? "she" pronoun) "her")
			      	      ((string-ci=? "it" pronoun) "its")
			      	      ((string-ci=? "we" pronoun) "our")
			      	      ((string-ci=? "they" pronoun) "their")
			      	)
			      )
			      ; if (indirect) object same as subject
			      ((equal? (cog-pred-get-argN eval-link 0) atom)
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

		(define (clone-helper sublink last-eval-link)
			(define n-index 0)
			(define old-oset (cog-outgoing-set sublink))
			(define (change-old atom)
				(cond ((cog-link? atom)
					; update the closest EvaluationLink as necessary
					(clone-helper atom (if (equal? 'EvaluationLink (cog-type atom)) atom last-eval-link))
				      )
				      (else
				      	; see if this node is in ns, and get its index
				      	(set! n-index (list-index (lambda (n) (equal? atom (nouns-list-get-noun n))) ns))
				      	
				      	; check if the noun should be replace with pronoun in ds
				      	(cond ((and n-index (list-ref ds n-index))
				      		; XXX do we need this new node to inherit from original?
				      		; XXX do it like anaphora resolution?
				      		(cog-new-node (cog-type atom)
			      				(finalize-pronoun atom (list-ref ps n-index) last-eval-link)
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
	
		; if ns empty, return original link (ie. the link has no noun)
		(if (null? ns)
			orig-link
			; clone, and if found a node within ns and ds is true, create a new node with new name ps
			(clone-helper orig-link (if (equal? 'EvaluationLink (cog-type orig-link)) orig-link '()))
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

				; makes it so that we will pass an empty subset if a link does not need to be changed
				; (eg. if a link does not contain a noun)
				(if (not start-index)
					(set! start-index (length nouns-subset))
				)

				; if we have reached the end of the subset
				(if (not end-index)
					(set! end-index (length nouns-subset))
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
				; (eg. if a chunk somehow has no noun)
				(if (not start-index)
					(set! start-index (length nouns-list))
				)
				; if we have reached the end of the set
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

	; TODO also insert anaphora for missing subjects/objects, nominal anaphora
	
	new-chunks
)


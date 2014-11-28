; loading additional dependency
(use-modules (oop goops))
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")

; -----------------------------------------------------------------------
; <noun-item> -- A class containing information on a noun
;
; An instance of this class will store all the information needed for
; processing a noun.
;
(define-class <noun-item> ()
	(nn #:init-keyword #:noun-node #:getter get-noun-node)		; the corresponding ConceptNode
	(lk #:init-keyword #:orig-link #:getter get-orig-link)		; the original link the node is part of
	(ai #:init-keyword #:atom-index #:getter get-atom-index)	; the node index within the link (named atom-index since node-index's short-hand will cause confusion)
	(li #:init-keyword #:link-index #:getter get-link-index)	; the index of the link within a chunk
	(ci #:init-keyword #:chunk-index #:getter get-chunk-index)	; the index of the chunk within a list of chunks
	(base-pronoun #:init-value "")					; the corresponding pronoun in its basic form
	(pronoun-safe #:init-value #f #:getter is-pronoun-safe? #:setter set-pronoun-safe!)	; indicate whether this usage can be changed to a pronoun
)

; -----------------------------------------------------------------------
; get-base-pronominal -- Get the basic form of the pronoun
;
(define-method (get-base-pronominal (ni <noun-item>))
	(define (determine-pronoun)
		(define word-inst (r2l-get-word-inst (get-noun-node ni)))
		(define is-pronoun (word-inst-has-attr? word-inst "pronoun"))
		(define is-singular (word-inst-has-attr? word-inst "singular"))
		(define is-human (word-inst-has-attr? word-inst "person"))
		(define is-male (and is-human (word-inst-has-attr? word-inst "masculine")))
		
		(define (rebase-pronoun orig)
			(cond ((regexp-exec (make-regexp "(I|me|my|myself)" regexp/icase) orig) "I")
			      ((regexp-exec (make-regexp "(you|your|yourself)" regexp/icase) orig) "you")
			      ((regexp-exec (make-regexp "(he|him|his|himself)" regexp/icase) orig) "he")
			      ((regexp-exec (make-regexp "(she|her|herself)" regexp/icase) orig) "she")
			      ((regexp-exec (make-regexp "(it|its|itself)" regexp/icase) orig) "it")
			      ((regexp-exec (make-regexp "(we|us|our|ourselves)" regexp/icase) orig) "we")
			      ((regexp-exec (make-regexp "(they|them|their|theirselves)" regexp/icase) orig) "they")
			)
		)
		
		; TODO check possession for "our", "his", "their", etc. and "ours", "hers", "theirs" etc.
		;    "our group" -> "we"
		;    "our cars" -> "they"
		; XXX "our cars" would likely be two seperate nodes in atomspace "us" and "car", possibly
		;     in EvaluationLink "possession", so instead of replacing "our car", we would be 
		;     replacing "car" with pronoun, and delete the "possession" link...
				
		(cond ; if already a pronoun, change it to the base form
		      (is-pronoun
			(rebase-pronoun (word-inst-get-word-str word-inst))
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

	; get & store the pronoun if not determined before
	(if (= 0 (string-length (slot-ref ni 'base-pronoun)))
		(slot-set! ni 'base-pronoun (determine-pronoun))
	)
	
	(slot-ref ni 'base-pronoun)
)

; -----------------------------------------------------------------------
; get-modified-pronominal -- Get the modified form of the pronoun
;
; The modified form will be based on the noun's usage (subject/object).
;
; XXX how to handle "mine", "hers", "theirs", etc.
;
(define-method (get-modified-pronomial (ni <noun-item>) (forms-list <list>))
	(define the-noun-node (get-noun-node ni))
	(define the-orig-link (get-orig-link ni))
	(define the-base-pronoun (get-base-pronominal ni))
	(define the-atom-index (get-atom-index ni))
	
	(define matched-subgraph)
	(define matched-base-index)
	(set-values! (matched-subgraph matched-base-index) (match-sentence-forms the-orig-link forms-list))
	
	; rebase the atom index to that of the matched-subgraph
	; the atom index is needed because of atom like
	;     (EvaluationLink (PredicateNode "punched") (ListLink (ConceptNode "I") (ConceptNode "I"))
	; where the same (ConceptNode "I") could be matched to "I" or "myself"
	(set! the-atom-index (- the-atom-index matched-base-index))
		
	; if matched a sentence form, the link has basic subject-verb-object structure
	(if matched-subgraph
		(cond ; if the node is at the subject position
		      ((= the-atom-index 1)
			the-base-pronoun
		      )
		      ; if (indirect) object same as subject
		      ((equal? (cog-pred-get-argN matched-subgraph 0) the-noun-node)
		      	(cond ((string-ci=? "I" the-base-pronoun) "myself")
		      	      ((string-ci=? "you" the-base-pronoun) "yourself")
		      	      ((string-ci=? "he" the-base-pronoun) "himself")
		      	      ((string-ci=? "she" the-base-pronoun) "herself")
		      	      ((string-ci=? "it" the-base-pronoun) "itself")
		      	      ((string-ci=? "we" the-base-pronoun) "ourselves")
		      	      ((string-ci=? "they" the-base-pronoun) "themselves")
		      	)
		      )
		      ; if (indirect) object different from subject
		      (else
			(cond ((string-ci=? "I" the-base-pronoun) "me")
			      ((string-ci=? "you" the-base-pronoun) "you")
			      ((string-ci=? "he" the-base-pronoun) "him")
			      ((string-ci=? "she" the-base-pronoun) "her")
			      ((string-ci=? "it" the-base-pronoun) "it")
			      ((string-ci=? "we" the-base-pronoun) "us")
			      ((string-ci=? "they" the-base-pronoun) "them")
			)
		      )
		)
		(cond ; check possession
		      ((and (cog-pred-get-pred the-orig-link)
		     	    (string=? (cog-name (cog-pred-get-pred the-orig-link)) "possession"))
		     	; XXX need additional checks for the possessor and the possessed
		     	;     the whole possession link can be discarded if the possessed can be a pronoun
		     	;     meaning an additional step is needed before inserting pronoun, changing the chunk
		      	(cond ((string-ci=? "I" the-base-pronoun) "my")
		      	      ((string-ci=? "you" the-base-pronoun) "your")
		      	      ((string-ci=? "he" the-base-pronoun) "his")
		      	      ((string-ci=? "she" the-base-pronoun) "her")
		      	      ((string-ci=? "it" the-base-pronoun) "its")
		      	      ((string-ci=? "we" the-base-pronoun) "our")
		      	      ((string-ci=? "they" the-base-pronoun) "their")
		      	)
		      )
		      ; not sentence-formed, not possession (such as about-rule, before-after-rule)
		      (else
			(cond ((string-ci=? "I" the-base-pronoun) "me")
			      ((string-ci=? "you" the-base-pronoun) "you")
			      ((string-ci=? "he" the-base-pronoun) "him")
			      ((string-ci=? "she" the-base-pronoun) "her")
			      ((string-ci=? "it" the-base-pronoun) "it")
			      ((string-ci=? "we" the-base-pronoun) "us")
			      ((string-ci=? "they" the-base-pronoun) "them")
			)
		      )
		)
	)
)

; -----------------------------------------------------------------------
; get-nominal -- Get the nominal anaphora
;
; Get some alternative nouns or phrases that represent the same object.
;
; TODO possibly ranked bases on size of incoming-set (# of usage)
;
(define-method (get-nominal (ni <noun-item>))
	; return a list of all possible nominal anaphora?
)


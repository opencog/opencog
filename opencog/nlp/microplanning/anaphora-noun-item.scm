(use-modules (oop goops))
(load-scm-from-file "../opencog/nlp/microplanning/helpers.scm")

(define-class <noun-item> ()
	(nn #:init-keyword #:noun-node #:getter get-noun-node)
	(lk #:init-keyword #:orig-link #:getter get-orig-link)
	(ai #:init-keyword #:atom-index #:getter get-atom-index)
	(li #:init-keyword #:link-index #:getter get-link-index)
	(ci #:init-keyword #:chunk-index #:getter get-chunk-index)
	(base-pronoun #:init-value "")
	(pronoun-safe #:init-value #f #:getter is-pronoun-safe? #:setter set-pronoun-safe!)
)

(define-method (get-base-pronominal (ni <noun-item>))
	(define (determine-pronoun)
		(define word-inst (r2l-get-word-inst (get-noun-node ni)))
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

	; get & store the pronoun if not determined before
	(if (= 0 (string-length (slot-ref ni 'base-pronoun)))
		(slot-set! ni 'base-pronoun (determine-pronoun))
	)
	
	(slot-ref ni 'base-pronoun)
)

; TODO still need to handle "mine", "hers", "theirs", etc.
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

(define-method (get-nominal (ni <noun-item>))
	; return a list of all possible nominal anaphora?
)


; loading additional dependency
(use-modules (oop goops))
(load "helpers.scm")

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
	(lexical-node #:init-value '())					; the OpenCog node that can be used as a replacement noun
	(lexical-safe #:init-value #f #:getter is-lexical-safe? #:setter set-lexical-safe!)	; indicate whether this usage can be changed to another lexical noun phrase
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
			      ((regexp-exec (make-regexp "(they|them|their|themselves)" regexp/icase) orig) "they")
			)
		)
		
		; TODO recognition of "our group" -> "we" and "our cars" -> "they"
				
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
; XXX how to handle "mine", "hers", "theirs", etc?  Seems these will mostly
; appear within some be-inheritance? (eg.  "That car is her car" becoming
; "That car is hers")  Can also appear in "That car of hers is yellow."
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
		     	; check if this is the possessor or the possessed
		     	(if (= the-atom-index 1)
			      	(cond ((string-ci=? "I" the-base-pronoun) "my")
			      	      ((string-ci=? "you" the-base-pronoun) "your")
			      	      ((string-ci=? "he" the-base-pronoun) "his")
			      	      ((string-ci=? "she" the-base-pronoun) "her")
			      	      ((string-ci=? "it" the-base-pronoun) "its")
			      	      ((string-ci=? "we" the-base-pronoun) "our")
			      	      ((string-ci=? "they" the-base-pronoun) "their")
			      	)
			      	; if this is the possessed, then the whole possession link can be removed
			      	'possessed-link-cancel
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
; get-lexical-node -- Return one lexical noun phrase's node.
;
; Find some alternative nouns or phrases that represent the same object,
; prefering nodes that inherit from this instance (ie. subset), so the
; meaning of the original sentence stays true.  Randomly return one with
; some weights.
;
; XXX might on some special occassion want the supersets?
;
(define-method (get-lexical-node (ni <noun-item>))
	(define (determine-lexical)
		(define the-noun-node (get-noun-node ni))
		; XXX also accept links that inherit the abstracted version?
		; since the anchor is also a ConceptNode, cog-get-link will return each link twice, so need to delete duplicates
		(define all-inheritances (delete-duplicates (cog-get-link 'InheritanceLink 'ConceptNode the-noun-node)))

		(receive (supersets subsets)
			 (partition (lambda (l) (equal? the-noun-node (gar l))) all-inheritances)

			; remove links in subsets that are not about a noun
			(set! subsets (filter (lambda (l) (word-inst-is-noun? (r2l-get-word-inst (gar l)))) subsets))
		
			; remove close to false or low confidence links base on TruthValue
			(set! subsets (filter (lambda (l) (and (> (tv-mean (cog-tv l)) 0.5) (> (tv-conf (cog-tv l)) 0.5))) subsets))

			(let* ((weights
				; calculate a weight for each link
				(map (lambda (l) (length (cog-incoming-set (gar l))) * (tv-mean (cog-tv l)) * (tv-conf (cog-tv l))) subsets))
			       (sorted-zip
				; sort bases on the weight
				(sort (zip weights (map gar subsets)) (lambda (s1 s2) (> (car s1) (car s2)))))
			       (appended-sorted-zip
				(if (null? sorted-zip)
					(list (list 1.0 (get-noun-node ni)))
					; add the original noun-node to the list with same weight as the head
					(cons (list (caar sorted-zip) (get-noun-node ni)) sorted-zip)
				))
			       (cdf-zip
				; calculate the cumulative density function to allow for weighted random selection
				(reverse
					(fold
						 (lambda (s lst)
						 	(if (null? lst)
						 		(cons s '())
						 		(cons (list (+ (car s) (caar lst)) (cadr s)) lst)
						 	)
						 )
						'()
						appended-sorted-zip
					)
				))
			       (r-num (random (* (car (last cdf-zip)) 1.0)))
			       ; find the first link with weight >= r-num from the CDF list
			      )

				(cadr (find (lambda (z) (>= (car z) r-num)) cdf-zip))
			)
		)
	)
	
	; get & store the lexical choice if not determined before
	(if (null? (slot-ref ni 'lexical-node))
		(slot-set! ni 'lexical-node (determine-lexical))
	)
	
	(slot-ref ni 'lexical-node)
)


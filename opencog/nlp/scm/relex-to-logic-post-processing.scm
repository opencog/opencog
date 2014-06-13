; -----------------------------------------------------------------------
; Check if a word instance has definite flag
(define (definite? word-inst)
	; get the list of DefinedLinguisticConceptNode connected to word-inst
	(define ling-concept-list (word-inst-get-attr word-inst))
	; for each item in ling-concept-list, want to check if name is "definite"
	(list-index (lambda (a-node) (string=? "definite" (cog-name a-node))) ling-concept-list)
)

; -----------------------------------------------------------------------
; Check if a word instance can be removed
(define (cleanable? word-inst)
	(or
		(and (not (definite? word-inst)) (word-inst-match-pos? word-inst "noun"))
		(word-inst-match-pos? word-inst "adj")
		(word-inst-match-pos? word-inst "adv")
	)
)

; -----------------------------------------------------------------------
; Main recursive function to rebuild the hypergraph and delete the old one
(define (rebuild ilink old new)
	; get all the nodes linked by this link
	(define old-list (cog-outgoing-set ilink))
	; find index of inst in old-list
	(define old-index (list-index (lambda (a-node) (equal? old a-node)) old-list))
	; create a copy of old-list with old-index replaced with new
	(define new-list (append (take old-list old-index)
				(list new)
				(drop old-list (+ 1 old-index))))
	; create a new link with the new node list
	(define new-link (apply cog-new-link (cog-type ilink) new-list))
	(define incoming-list (cog-incoming-set ilink))
	(define incoming-list-len (length incoming-list))

	(if (> incoming-list-len 0)
		(map rebuild incoming-list (make-list incoming-list-len ilink) (make-list incoming-list-len new-link))
		(delete-hypergraph ilink)
	)
)

; -----------------------------------------------------------------------
; Helper function to call the main recursive one
(define (rebuild-all old new)
	(define incoming-list (cog-incoming-set old))
	(map rebuild incoming-list (make-list (length incoming-list) old) (make-list (length incoming-list) new))
)

; -----------------------------------------------------------------------
; Clean up word instances from rules output if applicable
(define (clean-up-inst word-inst)
	; find the WordNode of this instance
	(define word-node (word-inst-get-lemma word-inst))
	; find the ConceptNode of this word instance
	(define word-inst-concept (cog-node 'ConceptNode (cog-name word-inst)))
	; find the ConceptNode of this word node
	(define word-node-concept (cog-node 'ConceptNode (cog-name word-node)))

	(cond
		((cleanable? word-inst)
			; for each graph with word-inst-concept, rebuild upward
			; replace all link with word-inst-concept with word-node-concept
			(rebuild-all word-inst-concept word-node-concept)
			; delete the newly created InheritanceLink linking the word with itself
			(delete-hypergraph (cog-link 'InheritanceLink word-node-concept word-node-concept))
		)
	)
)

; -----------------------------------------------------------------------
; Clean up the whole sentence as needed
(define (clean-up-parse parse-node)
	(define word-inst-list (parse-get-words parse-node))
	(map clean-up-inst word-inst-list)
)


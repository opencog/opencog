; -----------------------------------------------------------------------
; Get the corresponding ConceptNode
(define (cog-get-concept node)
	(if (null? node)
		'()
		(cog-node 'ConceptNode (cog-name node))
	)
)

; -----------------------------------------------------------------------
; Get the head link with no incoming set
(define (cog-get-root atom)
	(define iset (cog-incoming-set atom))
	(if (null? iset)
		(list atom)
		(append-map cog-get-root iset))
)

; split the list at each indices
(define (split-at-indices lst ind)
	(define (split-at-indices-helper lst ind)
		(if (null? ind)
			(list lst)
			(cons (drop lst (car ind)) (split-at-indices-helper (take lst (car ind)) (cdr ind)))
		)
	)

	(reverse (split-at-indices-helper lst (reverse ind)))
)


; -----------------------------------------------------------------------
; Main recursive function to build the new abstracted links
(define (rebuild ilink olds news)


	; get all the nodes linked by this link
	(define old-oset (cog-outgoing-set ilink))

	(define (replace-old candidate)
		(define ind (list-index (lambda (x) (equal? candidate x)) olds))
		; if no index found, this candidate is either a link or
		; does not need to be replaced
		(if (not ind)
			(if (cog-link? candidate)
				(rebuild candidate olds news)
				candidate
			)
			(list-ref news ind)
		)
	)

	(define new-oset (map replace-old old-oset))

	; create a new link with the new node list
	(apply cog-new-link (cog-type ilink) new-oset)
)

; -----------------------------------------------------------------------
; Find out which words can be abstracted and call the helper function to create them.
(define (create-abstract-version parse-node)
	(define word-inst-list (parse-get-words parse-node))
	(define word-list (map word-inst-get-lemma word-inst-list))

	; find the ConceptNode of each word instance and word node
	(define word-inst-concept-list (remove null? (map cog-get-concept word-inst-list)))
	(define word-concept-list (remove null? (map cog-get-concept word-list)))
	; get a list of InheritanceLink linking word instance with its word
	(define word-inheritance-list
		(map (lambda (inst word) (cog-link 'InheritanceLink inst word)) word-inst-concept-list word-concept-list)
	)

	; find all head links that included a word instance
	(define involvement-list (map cog-get-root word-inst-concept-list))
	(define involvement-cnt (map length involvement-list))

	; get word instances that only appear in one link (exclude the one
	; linking itself with the word node
	(define lone-word-inst-list
		(remove boolean? 
			(map (lambda (inst cnt) (if (<= cnt 2) inst #f))
				word-inst-concept-list
				involvement-cnt
			)
		)
	)

	(define lone-word-list
		(remove boolean? 
			(map (lambda (word cnt) (if (<= cnt 2) word #f))
				word-concept-list
				involvement-cnt
			)
		)
	)

	; all the links that involved a word, no duplicate
	(define cleaned-links
		(remove (lambda (x)
				(member x word-inheritance-list))
			(delete-duplicates (apply append involvement-list))
		)
	)

	(map (lambda (a-link) (rebuild a-link lone-word-inst-list lone-word-list))
		cleaned-links
	)
)



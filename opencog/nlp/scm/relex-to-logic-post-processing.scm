; -----------------------------------------------------------------------
; Get the corresponding ConceptNode or PredicateNode
(define (word-get-concept-or-predicate node)
	(define concept
		(if (null? node)
			'()
			(cog-node 'ConceptNode (cog-name node))
		)
	)

	(if (null? node)
		'()
		(if (null? concept)
			(cog-node 'PredicateNode (cog-name node))
			concept
		)
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

; -----------------------------------------------------------------------
; Generate a new unique name for a word
(define (create-unique-word-name word)
	(define (create-new-name w)
		(define tail-name (random-string 32))
		(string-append (cog-name w) "@" tail-name)
	)
	(define new-name (create-new-name word))

	(while (check-name? new-name 'ConceptNode)
		(set! new-name (create-new-name word))
	)
	new-name
)

; -----------------------------------------------------------------------
; Main recursive function to build the new abstracted links
(define (rebuild ilink old-new-pairs other-name-pairs)
	; get all the nodes linked by this link
	(define old-oset (cog-outgoing-set ilink))
	(define (replace-old candidate)
		(define new (assoc candidate old-new-pairs))
		; if this candidate is either a link or does not need to be replaced
		(if (not new)
			(if (cog-link? candidate)
				(rebuild candidate old-new-pairs other-name-pairs)
				(cog-new-node (cog-type candidate) (cdr (assoc candidate other-name-pairs)))
			)
			(cdr new)
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

	; find the ConceptNode/PredicateNode of each word instance and word node
	(define word-inst-concept-list (remove null? (map word-get-concept-or-predicate word-inst-list)))
	(define word-concept-list (remove null? (map word-get-concept-or-predicate word-list)))

	; get a list of InheritanceLink linking word instance with its word
	(define word-inheritance-list
		(map (lambda (inst word) (cog-link 'InheritanceLink inst word)) word-inst-concept-list word-concept-list)
	)

	; for each word instant, find a list of all head links that includes it
	(define word-involvement-list (map cog-get-root word-inst-concept-list))
	(define word-involvement-cnt (map length word-involvement-list))

	; get word instances that only appear in one link (exclude the one
	; linking itself with the word node), and their associated word
	(define lone-word-assoc-list
		(remove boolean? 
			(map (lambda (inst word cnt) (if (<= cnt 2) (cons inst word) #f))
				word-inst-concept-list
				word-concept-list
				word-involvement-cnt
			)
		)
	)

	; get word instances that are not lone word, and create a new unique name for them
	(define non-lone-word-assoc-list
		(remove boolean? 
			(map (lambda (inst word cnt) (if (> cnt 2) (cons inst (create-unique-word-name word)) #f))
				word-inst-concept-list
				word-concept-list
				word-involvement-cnt
			)
		)
	)

	; get all head links that contains one of the lone word
	(define lone-word-involvement-list (map cog-get-root (map car lone-word-assoc-list)))

	; all the links that involved a lone word, no duplicate
	(define cleaned-links
		(remove (lambda (x)
				(member x word-inheritance-list))
			(delete-duplicates (apply append lone-word-involvement-list))
		)
	)

	(map (lambda (a-link) (rebuild a-link lone-word-assoc-list non-lone-word-assoc-list))
		cleaned-links
	)
)



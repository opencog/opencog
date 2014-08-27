;
; post-processing.scm
;
; Assorted functions for post-processing relex2logic output.
;
; Copyright (c) 2014 William Ma <https://github.com/williampma>
;

; =======================================================================
; Helper utilities for post-processing.
; =======================================================================

; -----------------------------------------------------------------------
; word-get-r2l-node -- Retrieve corresponding R2L created node
;
; Given a WordInstanceNode created by RelEx, retrieve the corresponding
; ConceptNode or PredicateNode or NumberNode created by R2L helper
;
(define (word-get-r2l-node node)
	(define name
		(if (not (null? node))
			(cog-name node)
		)
	)

	(cond ((null? node) '())
		((not (null? (cog-node 'ConceptNode name))) (cog-node 'ConceptNode name))
		((not (null? (cog-node 'PredicateNode name))) (cog-node 'PredicateNode name))
		((not (null? (cog-node 'NumberNode name))) (cog-node 'NumberNode name))
		(else '())
	)
)

; -----------------------------------------------------------------------
; check-exception -- Check if 'link' should be excluded from markers post-processing
;
; Given a link, check to see if it should be ignored.  This could be a ReferenceLink
; or a link with any sub-node does not have the corresponding WordInstanceNode
; from RelEx (with some exceptions)
;
(define (check-exception link)
	(define words (cog-get-all-nodes link))
	; check if a node has no corresponding 'WordInstanceNode
	(define (check-word w)
		(and
			; exception that should not be considered as non-instance
			; TODO better node name for automatic checking possible
			(not (equal? 'VariableNode (cog-type w)))
			(not (equal? 'ReferenceNode (cog-type w)))
			(not (string=? "possession" (cog-name w)))
			(not (string=? "after" (cog-name w)))
			(not (string=? "before" (cog-name w)))
			(not (string=? "that" (cog-name w)))
			; the actual check
			(null? (cog-node 'WordInstanceNode (cog-name w)))
		)
	)
	(or (equal? 'ReferenceLink (cog-type link)) (find check-word words))
)

; -----------------------------------------------------------------------
; pairwise-combination -- Create all possible pairwise combination
;
; Given a list of lists, create all possible pairwise combination
; between lists.
;
(define (pairwise-combination sets)
	(define (create-with-index index rest)
		(map (lambda (r) (list index r)) rest))
	(define (recursive-helper index rest)
		(if (null? (cdr rest))
			(create-with-index index rest)
			(append (create-with-index index rest)
				(recursive-helper (car rest) (cdr rest))
			)
		)
	)
	
	(recursive-helper (car sets) (cdr sets))
)

; -----------------------------------------------------------------------
; lset-pairwise-intersection -- Pairwise intersection
;
; Given a list of lists, do pairwise intersection, keeping any items
; that appear in more than one list.
;
(define (lset-pairwise-intersection sets)
	(define all-pairs (pairwise-combination sets))
	(append-map (lambda (pair-list) (lset-intersection equal? (car pair-list) (cadr pair-list)))
			all-pairs)
)

; -----------------------------------------------------------------------
; member? -- Custom member? function to return the item instead of a list
;
; Same as 'member' but return the original item instead of a list.
;
(define (member? x lst)
	(if (member x lst)
		x
		#f
	)
)

; -----------------------------------------------------------------------
; definite? -- Check if a ConceptNode, check if it is DEFINITE
;
; Check if a word-inst ConceptNode has definite-rule applied.
;
(define (definite? word-concept-inst)
	(define definite (cog-node 'PredicateNode "definite"))
	(define llink (cog-link 'ListLink word-concept-inst))
	(and (not (null? definite))
		(not (null? llink))
		(not (null? (cog-link 'EvaluationLink definite llink))))
)

; -----------------------------------------------------------------------
; random-hex-string -- Generate a random string of hex
;
; Returns a random hex string of length 'str-length'.
;
(define (random-hex-string str-length) 
	(define alphanumeric "abcdef0123456789")
	(define str "")
	(while (> str-length 0)
		(set! str (string-append str (string (string-ref alphanumeric (random (string-length alphanumeric))))))
		(set! str-length (- str-length 1))
	)
	str
)

; -----------------------------------------------------------------------
; random-UUID -- Generate a new UUID version 4
;
; Returns UUID version 4 (ie, mostly just random hex with some fixed values)
;
(define (random-UUID)
	(define part1 (random-hex-string 8))
	(define part2 (random-hex-string 4))
	(define part3 (string-append "4" (random-hex-string 3)))
	(define reserve (string (string-ref "89ab" (random 4))))
	(define part4 (string-append reserve (random-hex-string 3)))
	(define part5 (random-hex-string 12))
	(string-append part1 "-" part2 "-" part3 "-" part4 "-" part5)
)

; -----------------------------------------------------------------------
; create-unique-word-name -- Generate a new unique name for a word
;
; Given a non-instanced word's ConceptNode, generate a new instance,
; appending @ UUID.
;
(define (create-unique-word-name word)
	(define (create-new-name w)
		(define tail-name (random-UUID))
		(string-append (cog-name w) "@" tail-name)
	)
	(define new-name (create-new-name word))

	(while (check-name? new-name 'ConceptNode)
		(set! new-name (create-new-name word))
	)
	new-name
)


; =======================================================================
; Post-processsing functions for markers created from pre-processing.
; =======================================================================

; -----------------------------------------------------------------------
; r2l-marker-processing -- Entry point of markers post-processing
;
; Call all markers' post-processing steps.
;
(define (r2l-marker-processing)
	(define results
		(append
			(marker-cleaner "allmarker" allmarker-helper)
			(marker-cleaner "maybemarker" maybemarker-helper)
		)
	)
	; helper function that prune away atoms that no longer exists
	; from subsequent cleaners, or atoms that are wrapped inside
	; another link
	(define (pruner x)
		(define deref-x (cog-atom (cog-handle x)))
		; if 'deref-x' is #<Invalid handle>, than both cog-node?
		; and cog-link? will return false
		(if (and (or (cog-node? deref-x) (cog-link? deref-x))
			 (null? (cog-incoming-set x)))
			x
			#f
		)
	)
	(filter-map pruner results)
)

; -----------------------------------------------------------------------
; allmarker-helper -- Helper function of allmarker post-processing
;
; The allmarker helper function for post-processing one specific
; allmarker.
;
; Given an allmarker of the form as 'orig-link':
;
;	EvaluationLink
;		PredicateNode "allmarker"
;		ListLink
;			ConceptNode noun_instance
;
; It creates:
;
;	ForAllLink
;		VariableNode "$X"
;		ImplicationLink
;			InheritanceLink "$X" noun_instance
;			AndLink
;				** links involving noun_instance **
;
; meaning all links involving noun_instance (except maybe other markers,
; or Inherit n n_inst) are included.  Each noun_instance will be
; replaced as (VariableNode "$X").
;
(define (allmarker-helper orig-link)
	(define listlink (car (cog-filter 'ListLink (cog-outgoing-set orig-link))))
	(define word (gar listlink))
	(define root-links (cog-get-root word))
	; get rid of links with non-instanced word
	(define clean-links (remove check-exception root-links))
	; helper recursive function to rebuild a link, replacing 'word' node with $X
	(define (rebuild-with-x link)
		(define old-oset (cog-outgoing-set link))
		(define (rebuild-helper atom)
			(if (cog-link? atom)
				(rebuild-with-x atom)
				(if (equal? atom word)
					(VariableNode "$X" df-node-stv)
					atom
				)
			)
		)
		(define new-oset (map rebuild-helper old-oset))

		(apply cog-new-link (append (list (cog-type link) (cog-tv link)) new-oset))
	)

	; rebuild each link, replace 'word' with (VariableNode "$X")
	(define final-links (map rebuild-with-x clean-links))
	(define results-list
		(list
			(ForAllLink df-link-stv
				(VariableNode "$X" df-node-stv)
				(ImplicationLink df-link-stv
					(InheritanceLink df-link-stv
						(VariableNode "$X" df-node-stv)
						word
					)
					; new rebuilt links
					(if (= (length final-links) 1)
						final-links
						(AndLink df-link-stv final-links)
					)
				)
			)
		)
	)

	; delete old rebuilt links
	(for-each purge-hypergraph clean-links)

	results-list
)

; -----------------------------------------------------------------------
; maybemarker-helper -- Helper function of maybemarker post-processing
;
; The maybemarker helper function for post-processing one specific
; maybemarker.
;
; Given an maybemarker of the form as 'orig-link':
;
;	EvaluationLink
;		PredicateNode "maybemarker"
;		ListLink
;			ConceptNode word_instance
;
; we find all root links containing 'word_instance' and change the
; confidence to 0.5.
;
(define (maybemarker-helper orig-link)
	(define listlink (car (cog-filter 'ListLink (cog-outgoing-set orig-link))))
	(define word (gar listlink))
	(define root-links (cog-get-root word))
	; get rid of links with non-instanced word
	(define clean-links (remove check-exception root-links))
	(define (change-tv l)
		(cog-set-tv! l (cog-new-stv 0.99 0.5))
	)

	; modify the TV
	(map change-tv clean-links)
)

; -----------------------------------------------------------------------
; marker-cleaner -- Common code for calling marker's helper function
;
; A general purpose marker function that calls a specific 'helper' function
; to clean each instance of a marker with 'name' in the atomspace, and
; delete them.
;
(define (marker-cleaner name helper)
	(define marker (cog-node 'PredicateNode name))
	(define (call-helper)
		; get the list of all unprocessed marker of type "name"
		(define marker-list (cog-get-link 'EvaluationLink 'ListLink marker))
		; call helper function to process them
		(define results-list (append-map helper marker-list))
		; delete the markers links and the marker itself
		(for-each purge-hypergraph marker-list)
		(cog-delete marker)
		; return the results
		results-list
	)

	(if (null? marker)
		'()
		(call-helper)
	)
)


; =======================================================================
; Functions to create partially and fully abstract version of the
; representations.
; =======================================================================

; -----------------------------------------------------------------------
; rebuild -- Main recursive function to build the new abstracted links
;
; A helper function for building a new version of 'ilink' with abstraction.
;
; 'old-new-pairs' contains pairs (old, new) of nodes where 'old' needs
; to be abstracted and 'new' is the non-instanced version. All 'old' in
; 'ilink' will be replaced by 'new'.
;
; 'other-name-triplets' contains triplets (orig, new, word) of nodes where
; 'orig' is an instanced word node, 'new' is the new instanced name, and
; 'word' is the non-instanced version.  For each 'orig', a new instanced
; node 'new' replaces it, and 'new' will inherit from 'word'. 
;
(define (rebuild ilink old-new-pairs other-name-triplets)
	; get all the nodes linked by this link
	(define old-oset (cog-outgoing-set ilink))
	(define (replace-old candidate)
		(define new (assoc candidate old-new-pairs))
		(define triplet (assoc candidate other-name-triplets))
		; if this candidate is either a link or does not need to be replaced
		(if (not new)
			(cond ((cog-link? candidate) (rebuild candidate old-new-pairs other-name-triplets))
				((not triplet) candidate)
				(else
					(if (equal? 'PredicateNode (cog-type candidate))
						(ImplicationLink (cog-new-node (cog-type candidate) (cadr triplet)) (caddr triplet))
						(InheritanceLink (cog-new-node (cog-type candidate) (cadr triplet)) (caddr triplet))
					)
					(cog-node (cog-type candidate) (cadr triplet))
				)
			)
			(cdr new)
		)
	)
	(define new-oset (map replace-old old-oset))

	; create a new link with the new node list
	(apply cog-new-link (cog-type ilink) new-oset)
)

; -----------------------------------------------------------------------
; create-abstract-version -- Create the abstracted representation
;
; Given a 'parse-node' of a parse, find out which words can be abstracted
; and call the helper function to create them. Both the partial and
; fully abstracted version are created.
;
(define (create-abstract-version parse-node)
	(define word-inst-list (parse-get-words parse-node))
	(define word-list (map word-inst-get-lemma word-inst-list))

	; find the ConceptNode/PredicateNode of each word instance and word node
	(define word-inst-concept-list (remove null? (map word-get-r2l-node word-inst-list)))
	(define word-concept-list (remove null? (map word-get-r2l-node word-list)))
	(define word-assoc-list (zip word-inst-concept-list word-concept-list))

	; for each word instant, find a list of all root links that includes it, and remove unneeded links
	(define word-involvement-messy-list (map cog-get-root word-inst-concept-list))
	(define word-involvement-intersection (lset-pairwise-intersection word-involvement-messy-list))
	(define word-involvement-cleaned-list  ; removing links with only one word involved
		(map (lambda (l) (filter-map (lambda (link) (member? link word-involvement-intersection)) l))
			word-involvement-messy-list
		)
	)
	(define word-involvement-cnt (map length word-involvement-cleaned-list))

	; get word instances that only appear in one link and their associated word
	; except words that are definite, which should not be cleaned
	(define lone-word-assoc-list
		(filter-map (lambda (inst word cnt) (if (and (= cnt 1) (not (definite? inst))) (cons inst word) #f))
			word-inst-concept-list
			word-concept-list
			word-involvement-cnt
		)
	)

	; get word instances that are not lone word, and create a new unique name for them;
	; note that the new name might not be needed, since two non-lone words can be linking to each
	; other with no lone-word involved (so we won't be creating InheritanceLink new-inst word here)
	(define non-lone-word-assoc-list
		(filter-map (lambda (inst word cnt) (if (or (> cnt 1) (definite? inst)) (list inst (create-unique-word-name word) word) #f))
			word-inst-concept-list
			word-concept-list
			word-involvement-cnt
		)
	)

	(define all-links
		(delete-duplicates (apply append word-involvement-cleaned-list))
	)

	; do the main creation for the partially abstract version
	(define partial
		(map (lambda (a-link) (rebuild a-link lone-word-assoc-list non-lone-word-assoc-list))
			all-links
		)
	)

	; do the main creation for the fully abstract version
	(define full
		(map (lambda (a-link) (rebuild a-link word-assoc-list '()))
			all-links
		)
	)

	(append partial full)	
)


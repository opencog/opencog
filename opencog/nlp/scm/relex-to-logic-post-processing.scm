;
; relex-to-logic-post-processing.scm
;
; Assorted functions for post-processing relex2logic output.
;

; =======================================================================
; Helper utilities for post-processing.
; =======================================================================

; -----------------------------------------------------------------------
; Get the corresponding ConceptNode or PredicateNode or NumberNode
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
; Helper function to check if a link contains a non-instance word
(define (check-non-instance link)
	(define words (cog-get-all-nodes link))
	(find (lambda (w) (null? (cog-node 'WordInstanceNode (cog-name w)))) words)
)

; -----------------------------------------------------------------------
; Get the root link with no incoming set
(define (cog-get-root atom)
	(define iset (cog-incoming-set atom))
	(if (null? iset)
		(list atom)
		(append-map cog-get-root iset))
)

; -----------------------------------------------------------------------
; Get all the nodes within a hypergraph
(define (cog-get-all-nodes link)
	(define oset (cog-outgoing-set link))
	(define (recursive-helper atom)
		(if (cog-link? atom)
			(cog-get-all-nodes atom)
			(list atom)
		)
	)

	(append-map recursive-helper oset)
)

; -----------------------------------------------------------------------
; Create all possible pairwise combination of the items
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
; Pairwise intersection, keeping any items that appear in more than one set
(define (lset-pairwise-intersection sets)
	(define all-pairs (pairwise-combination sets))
	(append-map (lambda (pair-list) (lset-intersection equal? (car pair-list) (cadr pair-list)))
			all-pairs)
)

; -----------------------------------------------------------------------
; Custom member? function to return the item instead of a list
(define (member? x lst)
	(if (member x lst)
		x
		#f
	)
)

; -----------------------------------------------------------------------
; Check if a word-inst has definite-rule applied
(define (definite? word-concept-inst)
	(define definite (cog-node 'PredicateNode "definite"))
	(define llink (cog-link 'ListLink word-concept-inst))
	(and (not (null? definite))
		(not (null? llink))
		(not (null? (cog-link 'EvaluationLink definite llink))))
)

; -----------------------------------------------------------------------
; Returns a random hex string of length 'str-length'.
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
; Returns UUID version 4
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
; Generate a new unique name for a word
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
; call all post-processing steps
(define (r2l-post-processing)
	(thatmarker-cleaner)
)

; -----------------------------------------------------------------------
; The thatmarker helper function for post-processing one specific
; thatmarker.
;
; Given sentence "I think that dogs attack angry cats.", and thatmarker
; in the form:
;
;	EvaluationLink
;		thatmarker
;		ListLink
;			think
;			attack
;
; We create:
;
;	EvaluationLink
;		that
;		ListLink
;			think
;			AndLink
;				EvaluationLink attack ListLink dog cat
;				InheritanceLink cat angry
;
; So basically, AndLink everything that directly or indirectly connects
; with the word "attack" (ie. the part of the sentence after "that")
;
(define (thatmarker-helper orig-link)
	(define listlink (car (cog-filter 'ListLink (cog-outgoing-set orig-link))))
	(define thatmarker (cog-node 'PredicateNode "thatmarker"))
	(define word1 (gar listlink))
	(define word2 (gdr listlink))
	
	; find all the words that link directly or indirectly with word2
	(define (get-all-connected-words)
		; helper function to get words that share the same hypergraph with 'word'
		(define (get-all-closely-connected-words word)
			(define all-links (cog-get-root word))
			; clean any links that include any nodes that are not instanced word
			; to avoid including other marker links, InheirtanceLink btwn word
			; and instance, etc
			(define cleaned-links (remove check-non-instance all-links))
			(delete-duplicates (append-map cog-get-all-nodes cleaned-links))
		)
		; the main recursive function to gather all indirectly connected words
		(define (get-all-connected-words word)
			(cond ((not (member? word connected-words))
				(set! connected-words (append (list word) connected-words))
				(for-each get-all-connected-words (get-all-closely-connected-words word))
				)
			)
		)
		; starts with word1 & thatmarker to avoid getting their connected words
		; then delete them afterward
		(define connected-words (list word1 thatmarker)) 
		(get-all-connected-words word2)
		(set! connected-words (delete thatmarker connected-words))
		(set! connected-words (delete word1 connected-words))
		connected-words
	)

	(define all-connected-words (get-all-connected-words))

	; for each word in connected-words, get the root link
	(define all-root-links (delete-duplicates (append-map cog-get-root all-connected-words)))
	
	; any links containing a non-instance word can be removed
	(define final-links
		(remove check-non-instance all-root-links)
	)

	; the final representation
	(list
		(EvaluationLink
			(PredicateNode "that")
			(ListLink
				word1
				(if (= (length final-links) 1)
					final-links
					(AndLink final-links)
				)
			)
		)
	)
)

; -----------------------------------------------------------------------
; The main thatmarker function that calls the helper to clean all
; thatmarker in the atomspace, and delete them
(define (thatmarker-cleaner)
	(define thatmarker (cog-node 'PredicateNode "thatmarker"))
	(define (call-helper)
		; get the list of all unprocessed thatmarker
		(define thatmarker-list (cog-get-link 'EvaluationLink 'ListLink thatmarker))
		; call helper function to process them
		(define results-list (append-map thatmarker-helper thatmarker-list))
		; delete the thatmarkers links and the thatmarker itself
		(for-each purge-hypergraph thatmarker-list)
		(cog-delete thatmarker)
		; return the results
		results-list
	)	

	(if (null? thatmarker)
		'()
		(call-helper)
	)
)


; =======================================================================
; Functions to create partially and fully abstract version of the
; representations.
; =======================================================================

; -----------------------------------------------------------------------
; Main recursive function to build the new abstracted links
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
; Find out which words can be abstracted and call the helper function to create them.
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


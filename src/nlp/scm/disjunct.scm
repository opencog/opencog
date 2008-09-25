scm
; 
; disjunct.scm
;
; Build lists of link-grammar disjuncts; update the SQL
; database counts with the results.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; =============================================================
; Callback-style disjuncts
;
(define (prt-stuff h) (display h) #f)

; callback-style disjunct stuff.
(define (cb-get-sentence-disjuncts sent-node)

	(define (dj-per-word word)
		(display word)
		; (display (cog-incoming-set word))
	
		#f
	)
	
	; loop over all the words in the sentence
	(cog-map-chase-link 'SentenceLink 'ConceptNode "" "" dj-per-word sent-node)
	
	#f
)

; Do it callback-style
; (cog-map-type cb-get-sentence-disjuncts 'SentenceNode)

; ===========================
; List-style disjuncts
;
; Return a list of all of the link-grammar links the word particpates in
(define (get-lgl word)

	(let* ((word-pairs (cog-filter-incoming 'ListLink word))
			)
		; (cog-filter-incoming
		(display "duuude: ")
		(display word-pairs)
		(newline)
	
	)
)

; Given a single sentence, process the disjuncts for that sentence
(define (list-get-sentence-disjuncts sent-node)

	; Get list of all words in the sentence
	(define (get-word-list sent)
		(cog-chase-link 'SentenceLink 'ConceptNode sent)
	)

	(for-each get-lgl (get-word-list sent-node))
)

; Given a list of sentences, process each sentence to extract disjuncts
(define (list-get-sentence-list-disjuncts sent-list)

	; Loop over a list of sentences, getting disjuncts for each sentence.
	(for-each list-get-sentence-disjuncts sent-list)
)

; Do it list-style
(list-get-sentence-list-disjuncts (cog-get-atoms 'SentenceNode))

; ===========================
; wire-style

(define (wire-it)

	; Create a wire to transport a stream of sentences
	(define sentences (make-wire))

	; More wires to transport various bits and pieces.
	(define word-instances (make-wire))
	(define word-nodes (make-wire))
	(define word-instance-pairs (make-wire))
	(define lg-connectors (make-wire))

	; Put the sentences on the wire
	(cgw-source-atoms sentences 'SentenceNode)

	; Get the incoming links.
	(cgw-follow-link sentences word-instances 'SentenceLink 'ConceptNode)
	
	; Get the word-nodes associated with the word-instances.
	; (cgw-follow-link word-instances word-nodes 'ReferenceLink 'WordNode)
	
	(cgw-filter-incoming word-instances word-instance-pairs 'ListLink)

	(cgw-follow-link word-instance-pairs lg-connectors
		'EvaluationLink 'LinkGrammarRelationshipNode)

	; print things out
	(wire-probe "conns" lg-connectors)
)

;===========================


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

;===========================
; Return a list of all of the link-grammar links the word particpates in
(define (get-lgl word)
	
	'()
)

(define (list-get-sentence-disjuncts sent-node)
	; Get list of all words in the sentence
	(define (get-word-list sent)
		(cog-chase-link 'SentenceLink 'ConceptNode sent)
	)

	(display (get-word-list sent-node))
)

; List-style disjuncts
(define (list-get-sentence-list-disjuncts sent-list)
	(for-each list-get-sentence-disjuncts sent-list)
)

; Do it list-style
(list-get-sentence-list-disjuncts (cog-get-atoms 'SentenceNode))

;===========================
; wire-style

(define (wire-it)

	; Create a wire to transport a stream of sentences
	(define sentences (make-wire))

	; More wires to transport various bits and peices.
	(define word-instances (make-wire))
	(define word-nodes (make-wire))
	(define misc-a (make-wire))
	(define misc-b (make-wire))
	(define misc-c (make-wire))
	(define misc-d (make-wire))
	(define misc-e (make-wire))

	; Put the sentences on the wire
	(cgw-source-atoms sentences 'SentenceNode)

	; Get the incoming links.
	(cgw-follow-link sentences word-instances 'SentenceLink 'ConceptNode)
	
	; Get the word-nodes associated with the word-instances.
	; (cgw-follow-link word-instances word-nodes 'ReferenceLink 'WordNode)
	
	; (cgw-filter-incoming word-instances misc-a 'ListLink)
	; (cgw-follow-link-pos word-instances misc-b 'ListLink 0 1)
	; (wire-fan-out misc-a misc-b misc-c)
	; (wire-fan-in misc-a misc-d misc-c)
	(cgw-splitter misc-a misc-b word-instances 'ListLink 0 0)
	;
	; (cgw-splitter misc-a word-instances misc-b 'ListLink 0 1)
	; (cgw-splitter misc-a misc-c misc-b 'ListLink 0 1)

	; print things out
	; (wire-probe "links" misc-a)
	; (wire-probe "back" misc-c)
	(display (wire-drain-count misc-b))
	; (wire-drain misc-d)
)

;===========================


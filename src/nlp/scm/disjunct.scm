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
	(define sentences (make-wire "sentences"))

	; A wire to transport sentence parts
	(define sentence-parts (make-wire "sentence-parts"))

	(define sentence-links (make-wire "sentence-links"))

	; Put the sentences on the wire
	(cgw-source-atoms sentences 'SentenceNode)

	; Get the incoming links.
	(cgw-incoming sentence-parts sentences)
	
	; Reject everything that is not a SentenceLink
	; (cgw-display-atom-type sentence-links sentence-parts)
	(cgw-filter-atom-type sentence-links sentence-parts 'SentenceLink)

	(wire-probe "sent-list" sentence-links)
)

;===========================


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


; List-style disjuncts
(define (list-get-sentence-disjuncts sent-node)

	; Get list of all words in the sentence
	(define word-list (cog-chase-link 'SentenceLink 'ConceptNode sent-node))
	(display word-list)

	#f
)


(cog-map-type cb-get-sentence-disjuncts 'SentenceNode)
(cog-map-type list-get-sentence-disjuncts 'SentenceNode)

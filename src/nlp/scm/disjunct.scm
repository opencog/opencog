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

(define (get-sentence-disjuncts sent-node)


	(define (dj-per-word word)
		; (display word)
		(display (cog-incoming-set word))
		#f
	)
	
	; loop over all the words in the sentence
	(cog-map-chase-link 'SentenceLink 'ConceptNode "" "" dj-per-word sent-node)
	
	#f
)


(cog-map-type get-sentence-disjuncts 'SentenceNode)

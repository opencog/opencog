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

	(define (triple-pred ?rel-type ?rel-value ?first-item ?second-item)
		(if (symbol? ?rel-type)
			(display "oh no not suported\n")
			(display ?rel-type) 
		)
		(if (symbol? ?rel-value)
			(display "duude rel-val symbol\n")
			(display "fixed-rel-val not supported\n")
		)
		(if (symbol? ?first-item)
			(display "duude first symbol\n")
			(display "fixed-first not supported\n")
		)
		(if (symbol? ?second-item)
			(display "duude second symbol\n")
			(display "fixed-second not supported\n")
		)
	)

	(define (dj-per-word word)
		; (display word)
		; (display (cog-incoming-set word))
		(define ?rel '())
		(define ?rword '())
		(triple-pred "LinkGrammarRelationshipNode" '?rel word '?rword)
	
		#f
	)
	
	; loop over all the words in the sentence
	(cog-map-chase-link 'SentenceLink 'ConceptNode "" "" dj-per-word sent-node)
	
	#f
)


(cog-map-type get-sentence-disjuncts 'SentenceNode)

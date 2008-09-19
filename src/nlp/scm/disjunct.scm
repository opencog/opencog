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
	
	(display sent-node)
	#f
)


(cog-map-type get-sentence-disjuncts 'SentenceNode)

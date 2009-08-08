;
; query.scm
; Question-answwering related code, a re-implementation of the C++
; code in scheme.
; Most of the code here is very specific to the RelEx representation
; of sentences, and thus is "fragile" if that represtation changes.
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
;
; Return #t if the node is a WH query word, else return #f
;
(define (is_qvar wrd)
	(if (eq? (cog-type wrd) 'DefinedLinguisticConceptNode)
		(let ((wstr (cog-name wrd)))
			(or (equal?  "who" wstr)
				(equal? "what" wstr)
				(equal? "when" wstr)
				(equal? "where" wstr)
				(equal? "why" wstr)
			)
		)
		#f
	)
)

; ---------------------------------------------------------------------
;
; Return #t if the word is a WH query word, else return #f
;
(define (is-word-a-query word-inst)
	(cog-map-chase-link
		'InheritanceLink 'DefinedLinguisticConceptNode
		is_qvar word-inst
	)
)


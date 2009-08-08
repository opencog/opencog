;
; query.scm
; Question-answwering related code, a re-implementation of the C++
; code in scheme.
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
;
; Return #t if the word is a WH query word, else return #f
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

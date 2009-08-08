;
; query.scm
;
; Question-answering related code, a re-implementation of the C++
; code in scheme.
; Most of the code here is very specific to the RelEx representation
; of sentences, and thus is "fragile" if that represtation changes.
;
; Linas Vepstas August 2009
;
; ---------------------------------------------------------------------
; is-qvar? node -- Return #t if the node is a WH query word, else return #f
;
(define (is-qvar? wrd)
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
; is-word-a-query? word-inst
; Return #t if the word-instance is a WH query word, else return #f
;
(define (is-word-a-query? word-inst)
	(cog-map-chase-link
		'InheritanceLink 'DefinedLinguisticConceptNode
		is-qvar? word-inst
	)
)

; ---------------------------------------------------------------------
;
; Create a list of all the query variables in a link
;
(define (find-vars atom-list)
	(define (fv atoms lst)
		(if (eq? atoms '())
			lst      ;; we're done, return
			(if (pair? atoms) ;; if its a list
				(append! (fv (car atoms) '()) (fv (cdr atoms) lst))
				(if (cog-link? atoms)
					(fv (cog-outgoing-set atoms) lst) ;; traverse outgoing set.
					(if (is-word-a-query? atoms) ;; single atom -- is it a query?
						(cons atoms lst)
						lst
					)
				)
			)
		)
	)
	(fv atom-list '())
)

; ---------------------------------------------------------------------
;
; Create a VarScopeLink hold the question to be answered.
;
(define (make-triple-question trip)

	(let* ((vars (find-vars trip))
			(var (if (pair? vars) (car vars) '()))
			)
		(VariableScopeLink
			var
			(AndLink
				trip
			)
			trip
		)
	)
)

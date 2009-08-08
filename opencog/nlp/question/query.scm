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
; find-wh-words list-of-links
;
; Create a list of all the word-instances representing a WH-query
; (what, who, where, etc.) give a list of links. Note that is is
; not a simple scan for these words in a sentence -- rather, the
; parser has already marked these words as query-words. This is 
; important, because WH_words can be used in sentences that are not
; questions (for example: "That is what I said." is not a WH-question).
;
(define (find-wh-words atom-list)
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
			(ImplicationLink
				(AndLink
					trip
				)
				trip
			)
		)
	)
)

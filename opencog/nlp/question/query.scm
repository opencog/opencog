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
		(cond
			; If its a query word, append it to the list
			((cog-node? atoms)
				(if (is-word-a-query? atoms)
					(cons atoms lst)
					lst
				)
			)

			; If its a link, scan its outgoing set
			((cog-link? atoms)
				(fv (cog-outgoing-set atoms) lst)
			)

			; If its a list then scan the list
			((pair? atoms)
				(append! (append-map! find-wh-words atoms) lst)
			)
		)
	)
	(fv atom-list '())
)

; ---------------------------------------------------------------------
; Create a copy of a hypergraph, replacing any WH-words with
; VariableNodes

(define (replace-wh-words atoms)
	(cond
 		; If its a query word, replace by variable
		((cog-node? atoms)
			(if (is-word-a-query? atoms)
				(VariableNode (cog-name atoms))
				atoms
			)
		)
		; If its a link, create a new link, after substituting
		((cog-link? atoms)
			(cog-new-link 
				(cog-type atoms) 
				(cog-tv atoms)
				(replace-wh-words (cog-outgoing-set atoms))
			)
		)
		; If its a list, create a new list w/ substitutions
		((pair? atoms)
			(map replace-wh-words atoms)
		)
	)
)

; ---------------------------------------------------------------------
;
; Create a VarScopeLink hold the question to be answered.
;
(define (make-triple-question trip)

	(let* ((vars (find-wh-words trip))
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

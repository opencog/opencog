;
; query.scm
;
; Question-answering related code, a re-implementation of the C++
; code in scheme.
; XXX Most of this code is just plain wrong, its based on a bad design
; point (the same bad design that the C++ code had).   Please see 
; the file triples/README-question for the correct design.
;
; XXX this whole file should be deleted as soon as dependencies to the
; chatbot are cut.
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
; important, because WH-words can be used in sentences that are not
; questions (for example: "That is what I said." is not a WH-question).
;
(define (find-wh-words atom-list)
	(filter-hypergraph is-word-a-query? atom-list)
)

; ---------------------------------------------------------------------
; replace-wh-words atoms
;
; Create a copy of a hypergraph, replacing any WH-words with
; VariableNodes (of the same name as the WH-words)
; XXXX bad don't do this.
;
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
; Create a VarScopeLink holding a triple to be matched
;
(define (make-triple-question trip)
	(define (find-vars atoms)
		(define (is-var? atom) 
			(eq? 'VariableNode (cog-type atom))
		)
		(filter-hypergraph is-var? atoms)
	)

	(let* ((ques (replace-wh-words trip))
			(vars (find-vars ques))
			)
		; if variables were found, then assume that only
		; one variable was found.
		(if (pair? vars)
			(let ((var (car vars)))
				(VariableScopeLink
					var
					(ImplicationLink
						ques
						var
					)
				)
			)
			'() ; no vars were found, return empty list.
		)
	)
)

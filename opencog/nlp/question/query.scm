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
	(filter-hypergraph is-word-a-query? atom-list)
)

; ---------------------------------------------------------------------
; filter-hypergraph pred? atom-list
;
; Given a list of atoms, and a scheme-predicate pred?, return a
; list of atoms that satisfy the scheme-predicate.  This is not
; a simple srfi-1 'filter', rather, it traverses the hypergraph,
; applying the predicate to the subgraphs.
;
; In the current implementation, the scheme-rpdicate is assumed to 
; select only for Nodes.  
;
(define (filter-hypergraph pred? atom-list)
	(define (fv atoms lst)
		(cond
			; If its a query word, append it to the list
			((cog-node? atoms)
				(if (pred? atoms)
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
				(append! 
					(append-map! 
						(lambda (x) (filter-hypergraph pred? x)) atoms) 
					lst
				)
			)
		)
	)
	(fv atom-list '())
)

; ---------------------------------------------------------------------
; replace-wh-words atoms
;
; Create a copy of a hypergraph, replacing any WH-words with
; VariableNodes (of the same name as the WH-words)
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

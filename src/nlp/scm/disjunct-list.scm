scm
; 
; disjunct-list.scm
;
; Build lists of link-grammar disjuncts; update the SQL
; database counts with the results.
;
; This is also a part of a test exploring different coding styles.
; Similar fuunctionality is (paritally) implemented in disjunct.scm,
; but using a different approach.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
(use-modules (srfi srfi-1))

; =============================================================
; List-style disjuncts
; ---------------------------------------------------------------------
; Given a word, and the sentence in which the word appears, return
; a list of the ling-grammar relations in which the word appears. 
; The relations are sorted in sentence word-order.
;
; Sort a list of link-grammar relations, according to the
; order in which they appear in a sentence
;
(define (ldj-sort-rels word sent-node rel-list)

	; rel-list is a list of the link-grammar relations.
	(let ((snt-wrds (sentence-get-words sent-node)))

		; Compare two link-grammar relations, and determine thier sentence
		; word order.
		(define (wless? rel-a rel-b)

			; Return the index of the word in a sentence
			; Caution! ice-9/boot-9.scm loads a *different* list-index than
			; that defined in srfi-1. We want the srfi-1 variant.
			(define (windex wrd)
				(if (equal? "LEFT-WALL" (cog-name wrd))
					-1
					(list-index (lambda (w) (equal? wrd w)) snt-wrds)
				)
			)

			(let ((word-a (cog-pred-get-partner rel-a word))
					(word-b (cog-pred-get-partner rel-b word))
				)
				(< (windex word-a) (windex word-b))
			)
		)
		(sort rel-list wless?)
	)
)

; ---------------------------------------------------------------------
; Given a list of link-grammar relation hypergraph, return a string 
; listing all of the link-grammar relations. That is, given a list
; of items of the form
;
;    EvaluationLink
;        LinkGrammarRelationshipNode
;        ListLink
;            ConceptNode
;            ConceptNode
;
; it will return a string holding the names of the 
; LinkGrammarRelationshipNode's
;
(define (ldj-make-disjunct sorted-rels)

	; Given a single opencog predicate (EvaluationLink) triple
	; containing a link-grammar relation, just return the relation name,
	; as a string.
	(define (get-name rel)
		(cog-name (car (cog-filter-outgoing 'LinkGrammarRelationshipNode rel)))
	)

	; Given a list of names, create a string, padding it with blanks.
	(define (mk-dj-string name-list str)
		(if (null? name-list)
			str
			(mk-dj-string (cdr name-list) (string-append str (car name-list) " "))
		)
	)

	(mk-dj-string (map get-name sorted-rels) "")
)

; ---------------------------------------------------------------------
;
; Return a list of all of the link-grammar links the word particpates in
(define (ldj-get-lg-rels word rel-list)
	(define (is-word-in-rel? word rel)
		(let* ((lnk (cog-filter-outgoing 'ListLink rel))
				(wds (cog-outgoing-set (car lnk))) 
			)
			(cond
				((equal? word (car wds)) #t)
				((equal? word (cadr wds)) #t)
				(else #f)
			)
		)
	)
	(filter (lambda (rel) (is-word-in-rel? word rel)) rel-list)
)

; Assemble all the disjuncts this one word participaes in.
(define (ldj-process-disjunct word rel-list sent-node)

	; Return the word string associated with the word-instance
	(define (get-word-str word-inst)
		(cog-name (car (get-word word-inst)))
	)

	(let* ((dj-rels (ldj-get-lg-rels word rel-list))
			(sorted-dj-rels (ldj-sort-rels word sent-node dj-rels))
		)

(display "Word: ")
(display (get-word-str word))
(display " -- ")
(display (ldj-make-disjunct sorted-dj-rels))
(display "\n")
	)
)

; Given a single parse, process the disjuncts for that parse
(define (ldj-process-parse word-list parse-node sent-node)
	(let ((rel-list (parse-get-lg-relations parse-node)))
		(for-each
			(lambda (word) (ldj-process-disjunct word rel-list sent-node))
			word-list
		)
	)
)

; Process a single sentence
(define (ldj-process-sentence sent-node)
	(let ((word-list (sentence-get-words sent-node)))
		(for-each
			(lambda (prs) (ldj-process-parse word-list prs sent-node))
			(sentence-get-parses sent-node)
		)
	)
)

; Process each of the sentences in a document
(define (ldj-process-document doco)
	(for-each ldj-process-sentence (document-get-sentences doco))
)

(define (ldj-it) 
	(for-each ldj-process-document (cog-get-atoms 'DocumentNode))
)
; =====================================================================

.
exit

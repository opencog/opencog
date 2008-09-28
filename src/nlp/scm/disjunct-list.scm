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
; Given a list of relations, sort the relations in "sentence order"
; (or "parse order"). The "sent-node" argument provides the 
; sentence by whitch to sort the relations, and the "word" argument
; gives the word relative to which the sort should be done.
; So, for example, given 
;
;        +---Os---+
;        |    +-Ds+
;        |    |   | 
;     heard.v a dog.
;
; and the word "dog" and the relations "Ds Os", this will return "Os Ds"
; because "heard" comes before "a", and so "Os" comes before "Ds".
;
(define (ldj-sort-rels word parse-node rel-list)

	; rel-list is a list of the link-grammar relations.
	(let ((snt-wrds (parse-get-words parse-node)))

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
; Given a list of link-grammar relation hypergraphs, return a string 
; listing all of the link-grammar relations. That is, given a list
; of items of the form
;
;    EvaluationLink
;        LinkGrammarRelationshipNode
;        ListLink
;            ConceptNode
;            ConceptNode
;
; this routine will return a string holding the names of the 
; LinkGrammarRelationshipNode's
;
(define (ldj-make-disjunct-string sorted-rels)

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
; Return a list of all of the link-grammar links the word particpates in
;
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

; ---------------------------------------------------------------------
; Assemble all the disjuncts this one word participates in.
; That is, given a word, and a list of *all* of the relations in the
; sentence, this will extract only those relations that this word
; participates in and sort them in sentence order.
;
(define (ldj-get-disjuncts word rel-list parse-node)

	(ldj-sort-rels word parse-node 
		(ldj-get-lg-rels word rel-list)
	)
)

; ---------------------------------------------------------------------
; Process a disjunt -- stuff into database, whatever.
(define (ldj-process-disjunct word rel-list parse-node)

	; Return the word string associated with the word-instance
	(define (get-word-str word-inst)
		(cog-name (car (get-word word-inst)))
	)

(display "Word: ")
(display (get-word-str word))
(display " -- ")
(display (ldj-make-disjunct-string (ldj-get-disjuncts word rel-list parse-node)))
(display "\n")
)

; ---------------------------------------------------------------------
; Given a single parse, process the disjuncts for that parse
;
(define (ldj-process-parse parse-node)
	(let ((rel-list (parse-get-lg-relations parse-node))
			(word-list (parse-get-words parse-node))
		)
(display rel-list)
;		(for-each
;			(lambda (word) (ldj-process-disjunct word rel-list parse-node))
;			word-list
;		)
	)
)

; ---------------------------------------------------------------------
; Process a single sentence
;
(define (ldj-process-sentence sent-node)
	(for-each ldj-process-parse (sentence-get-parses sent-node))
)

; ---------------------------------------------------------------------
; Process each of the sentences in a document
;
(define (ldj-process-document doco)
	(for-each ldj-process-sentence (document-get-sentences doco))
)

(define (ldj-it) 
	(for-each ldj-process-document (cog-get-atoms 'DocumentNode))
)
; =====================================================================

.
exit

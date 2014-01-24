; 
; disjunct-list.scm
;
; Build lists of link-grammar disjuncts (sets of word connectors).
; Given a parsed sentence, the routines here will determine the 
; disjuncts that were used by link-grammar to perform the parse.
; The net result is a string containing the disjunct.  The string
; will be a simple, space-separated list of connectors. A "connector"
; is a link-grammar link relations, with an appended + or - indicating
; whether the connection is to the right or left.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
(use-modules (srfi srfi-1))
;
; =====================================================================
; ---------------------------------------------------------------------
; Given a list of relations, sort the relations in "sentence order"
; (or "parse order"). The "sent-node" argument provides the 
; sentence by which to sort the relations, and the "word" argument
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
; listing all of the link-grammar connectors. A "connector" is a 
; link-grammar link (relation), together with a direction (to the left,
; or to the right) which specifies which direction the connector plugs
; in.
;
; That is, given a list of items of the form
;
;    EvaluationLink
;        LinkGrammarRelationshipNode
;        ListLink
;            WordInstanceNode
;            WordInstanceNode
;
; this routine will return a string holding the names of the 
; LinkGrammarRelationshipNode's, appended with a + or - to indicate
; which direction the relation went in.
;
(define (ldj-make-disjunct-string word sorted-rels)

	; Given a single opencog predicate (EvaluationLink) triple
	; containing a link-grammar relation, just return the relation name,
	; as a string.
	(define (get-name rel)
		(cog-name (car (cog-filter 'LinkGrammarRelationshipNode (cog-outgoing-set rel))))
	)

	; Get the direction of the link ...
	; It is either to the left(-) or right(+)
	(define (get-direction rel)
		(if (equal? word
			(car (cog-outgoing-set (car (cog-filter 'ListLink (cog-outgoing-set rel))))))
			"+" "-"
		)
	)

	(define (get-nd rel)
		(string-append (get-name rel) (get-direction rel))
	)

	; Given a list of names, create a string, padding it with blanks.
	(define (mk-dj-string name-list str)
		(if (null? name-list)
			str
			(mk-dj-string (cdr name-list) (string-append str (car name-list) " "))
		)
	)

	(mk-dj-string (map get-nd sorted-rels) "")
)

; ---------------------------------------------------------------------
; Return a list of all of the link-grammar links the word particpates in
;
(define (ldj-get-lg-rels word)
	(cog-get-pred word 'LinkGrammarRelationshipNode)
)

; ---------------------------------------------------------------------
; Assemble all the connectors this one word participates in.
; That is, given a word, and a list of *all* of the relations in the
; sentence, this will extract only those relations that this word
; participates in and sort them in sentence order.
;
(define (ldj-get-connectors word parse-node)

	(ldj-sort-rels word parse-node 
		(ldj-get-lg-rels word)
	)
)

; =====================================================================

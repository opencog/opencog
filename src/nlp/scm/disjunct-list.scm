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
; Similar to get-lg-rels, but sorted.
;
(define (get-lg-rels-sorted word sent-node)

	; rel-list is a list of the link-grammar relations.
	(let ((rel-list (get-lg-rels word)))

		; Compare two link-grammar relations, and determine thier sentence
		; word order.
		(define (wless? rel-a rel-b)

			; Return the index of the word in a sentence
			; Caution! ice-9/boot-9.scm loads a *different* list-index than
			; that defined in srfi-1. We want the srfi-1 variant.
			(define (windex wrd)
				(if (equal? "LEFT-WALL" (cog-name wrd))
					-1
					(list-index (lambda (w) (equal? wrd w)) (sentence-get-words sent-node))
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
; Return a string listing all of the link-grammar relations for the
; word in the sentence. This string will be in proper sorted order,
; according to the appearence of the words in the sentence word-order.
(define (make-disjunct sorted-rels)

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
; Process the disjuncts for a single word in a sentence
;
(define (process-disjunct word sent-node)

	; Return the word string associated with the word-instance
	(define (get-word word-inst)
		(cog-name (car (cog-chase-link 'ReferenceLink 'WordNode word-inst)))
	)
(display "Word: ")
(display (get-word word))
(display " -- ")
(display (make-disjunct (get-lg-rels-sorted word sent-node)))
(display "\n")
)

; XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXx
; XXX Evertyhing above the X's is subtly wrong in various ways.
;
;
; Return a list of all of the link-grammar links the word particpates in
(define (get-lg-rels word rel-list)
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
(define (ldj-process-disjunct word rel-list)
	(display (get-lg-rels word rel-list))
)

; Given a single parse, process the disjuncts for that parse
(define (ldj-process-parse word-list parse-node)
	(let ((rel-list (parse-get-lg-relations parse-node)))
		(for-each
			(lambda (word) (ldj-process-disjunct word rel-list))
			word-list
		)
	)
)

; Process a single sentence
(define (ldj-process-sentence sent-node)
	(let ((word-list (sentence-get-words sent-node)))
		(for-each
			(lambda (prs) (ldj-process-parse word-list prs))
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

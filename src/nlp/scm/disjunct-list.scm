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
(use-modules (dbi dbi))
;
;; username is 'linas' in this example, passwd is 'asdf' and the
;; databasse is 'lexat', running on the local machine
;; The postgres server is at port 5432, which can be gotten from
;; /etc/postgresql/8.3/main/postgresql.conf
;; 
(define db-connection
	(dbi-open "postgresql" "linas:asdf:lexat:tcp:localhost:5432"))

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
(define (ldj-make-disjunct-string word sorted-rels)

	; Given a single opencog predicate (EvaluationLink) triple
	; containing a link-grammar relation, just return the relation name,
	; as a string.
	(define (get-name rel)
		(cog-name (car (cog-filter-outgoing 'LinkGrammarRelationshipNode rel)))
	)

	; Get the direction of the link -- 
	; It is either to the left(-) or right(+)
	(define (get-direction rel)
		(if (equal? word
			(car (cog-outgoing-set (car (cog-filter-outgoing 'ListLink rel)))))
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
; Assemble all the disjuncts this one word participates in.
; That is, given a word, and a list of *all* of the relations in the
; sentence, this will extract only those relations that this word
; participates in and sort them in sentence order.
;
(define (ldj-get-disjuncts word parse-node)

	(ldj-sort-rels word parse-node 
		(ldj-get-lg-rels word)
	)
)

; ---------------------------------------------------------------------
; Process a disjunt -- stuff into database, whatever.
(define (ldj-process-disjunct word parse-node)

	; return the parse score for this parse.
	(define (parse-get-score pn)
		(assq-ref (cog-tv->alist (cog-tv pn)) 'confidence)
	)

	(let ((iword (word-inst-get-inflected-word-str word))
			(djstr (ldj-make-disjunct-string word (ldj-get-disjuncts word parse-node)))
			(score (parse-get-score parse-node))
			(row #f)
		)

		; Generic table update frame
		; select-str and insert-str are static strings;
		; while update-proc should return an update string.
		(define (update-table select-str update-proc insert-str)
			(dbi-query db-connection select-str)

			(set! row (dbi-get_row db-connection))
			(if row
				(begin
					; flush the rows, else the update will fail.
					(dbi-get_row db-connection)
					(dbi-query db-connection (update-proc row))
				)
				(dbi-query db-connection insert-str)
			)
			; Flush the db status
			(set! row (dbi-get_row db-connection))
		)

		; This routine will update the marginal table.
		;
		; The inflected table has the form:
		; CREATE TABLE InflectMarginal (
		;    inflected_word TEXT NOT NULL UNIQUE,
		;    count FLOAT,
		;    probability FLOAT )
		;
		(define (update-marginal-table i-word p-score)

			(define (update-proc srow)
				(let ((up-score (+ p-score (assoc-ref srow "count"))))
					(string-append
						"UPDATE InflectMarginal SET count = "
						(number->string up-score)
						" WHERE inflected_word = '" i-word "'"
					)
				)
			)

			(update-table
				(string-append
					"SELECT * FROM InflectMarginal WHERE inflected_word='"
					i-word "'"
				)
				update-proc
				(string-append
					"INSERT INTO InflectMarginal (inflected_word, count) VALUES ('"
					i-word "', " (number->string p-score) ")"
				)
			)
		)

		; Update the disjunct table
		; The disjunct table has the form:
		;
		; CREATE TABLE Disjuncts (
		;    inflected_word TEXT NOT NULL,
		;    disjunct TEXT NOT NULL,
		;    count FLOAT,
		;    cond_probability FLOAT
		; );
		;
		(define (update-disjunct-table i-word disj-str p-score)
			(define (update-proc srow)
				(let ((up-score (+ p-score (assoc-ref srow "count"))))
					(string-append
						"UPDATE Disjuncts SET count = "
						(number->string up-score)
						" WHERE inflected_word = '" i-word
						"' AND disjunct = '" disj-str "'"
					)
				)
			)

			(update-table
				(string-append "SELECT * FROM Disjuncts WHERE inflected_word='"
					i-word "' AND disjunct = '" disj-str "'"
				)
				update-proc

				(string-append
					"INSERT INTO Disjuncts (inflected_word, disjunct, count) VALUES ('"
					i-word "', '" disj-str "', " (number->string p-score) ")"
				)
			)
		)

		(update-marginal-table iword score)
		(update-disjunct-table iword djstr score)

		; (display "Word: ")
		; (display iword)
		; (display " -- ")
		; (display djstr)
		; (display score)
		; (display "\n")
	)
)

; ---------------------------------------------------------------------
; Given a single parse, process the disjuncts for that parse
;
(define (ldj-process-parse parse-node)
	(let ( (word-list (parse-get-words parse-node)))
		(for-each
			(lambda (word) (ldj-process-disjunct word parse-node))
			word-list
		)
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

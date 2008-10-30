scm
; 
; stats-collection.scm
;
; Update the SQL database with counts of disjuncts, word senses, etc.
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

; ---------------------------------------------------------------------
; Generic SQL table update framework. Given some data, we want to 
; create a new database record, if it does not already exist; otherwise
; we want to update the existing record.  There's no convenient way to 
; do this in SQL, so its done "manually" here.
;
; select-str and insert-str are static strings; while update-proc should
; return an update string. select-str should select for a record,
; insert-str should insert a record.  update-proc is passed the record
; found by select; if no record was found, then the insrt-str is run.
; This framework assumes that the result of select is either zero, or
; one record; it will (silently) break if there are two or more.
;
(define (sql-update-table select-str update-proc insert-str)
	(define row #f)
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

; ---------------------------------------------------------------------
; Process a disjunct -- put the "plain" disjunct into the database.
; This is the "simple" routine, in that it only updates the simple
; word-disjunt pair database; it ignores word seneses.
;
; Arguments:
; iword - inflected word
; djstr - disjunct string
; score - parse score
;
(define (ldj-process-disjunct-simple iword djstr score)

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

		(sql-update-table
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

		(sql-update-table
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
)

; ---------------------------------------------------------------------
; Process a disjunct -- stuff into database, whatever.
(define (ldj-process-disjunct word parse-node)

	; return the parse score for this parse.
	(define (parse-get-score pn)
		(assq-ref (cog-tv->alist (cog-tv pn)) 'confidence)
	)

	(let ((iword (word-inst-get-inflected-word-str word))
			(djstr (ldj-make-disjunct-string word (ldj-get-connectors word parse-node)))
			(score (parse-get-score parse-node))
		)

		(ldj-process-disjunct-simple iword djstr score)

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
	(let ((word-list (parse-get-words parse-node)))
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
	(InheritanceLink doco (DistinguishedNode "#LDJ_completed"))
)

; ---------------------------------------------------------------------
; Return all of the documents for which WSD has been completed,
; but disjunct processing has not been.
;
; XXX The method used here, of tagging documents with "finished"
; tags, is rather ad-hoc, and is meant to be a stop-gap until
; opencog offers some way of defining a processing pipeline.
;
(define (ldj-find-docs)

	; ldj-not-done? returns true if doco not in the "ldj completed" list.
	(define (ldj-not-done? doco)
		(define (gotit? d2)
			(if (equal? doco d2) #t #f)
		)
		(not
			(any gotit?
				(cog-chase-link 'InheritanceLink 'DocumentNode 
					(DistinguishedNode "#LDJ_completed")
				)
			)
		)
	)
	; Look for WSD_completed documents that are not marked LDJ_completed
	(filter! ldj-not-done?
		(cog-chase-link 'InheritanceLink 'DocumentNode
			(DistinguishedNode "#WSD_completed")
		)
	)
)

; ---------------------------------------------------------------------
; Process every document for which disjunct processing has not
; yet been done.
(define (ldj-process) 
	(for-each ldj-process-document (ldj-find-docs))
)

; =====================================================================
.
exit

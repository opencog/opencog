#! /usr/bin/env guile
!#
;
; Atomspace deduplication repair script
;
; Due to bugs, the SQL backend can end up with multiple copies of
; atoms. This script will find them, merge them, and sum the counts
; on the associated count truth values.  Its up to you to recompute
; anything else.
;
; This script eliminates duplicate word-pairs only.
; It is semi-manual.

(use-modules (dbi dbi))
(use-modules (srfi srfi-1))

; The uuid of the ANY LG type.  That is, the uuid of the
; LinkGrammarRelationshipNode "ANY"
(define uuid-of-any 250)


(define conxion
       ; (dbi-open "postgresql" "linas:asdf:en_pairs:tcp:localhost:5432"))
       (dbi-open "postgresql" "linas:asdf:en_pairs:socket:/var/run/postgresql"))

(display conxion) (newline)

; --------------------------------------------------------------
(define (look-for-dupes query colm)
"
  look-for-dupes -- Look for duplicate atoms
  query should be the SQL query to perform
  colm should be the string column name; either 'name' or 'outgoing'

  Returns a list of the duplicate entries found for 'colm'
"
	(define word-set (make-hash-table 100123))
	(define dupe-list (list))
	(define word-count 0)
	(define row #f)

	(dbi-query conxion query)

	(display "Duplicate search connection status: ")
	(display (dbi-get_status conxion)) (newline)

	; Loop over table rows
	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the column value
		(let* (
			(word (cdr (assoc colm row)))
			(uuid (cdr (assoc "uuid" row)))
			(wuid (hash-ref word-set word)))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; Have we seen this item previously?
			(if wuid
				(begin
					(display "Oh no! Duplicate!! ") (display word) 
					(display " prid=") (display wuid)
					(display " uuid=") (display uuid)
					(newline)
					(set! dupe-list (cons word dupe-list))
				)
				(hash-set! word-set word uuid)
			)
			; (display word) (newline)
			(set! row (dbi-get_row conxion))
		)
	)

	(display "Count was ") (display word-count) (newline)
	dupe-list
)
; --------------------------------------------------------------

; The duplicate-pair-list will consist of word-pairs
; (acutally, WordNode uuid-pairs) that appear in more
; than one ListLink (they should be unique...)
(define duplicate-pair-list
	(look-for-dupes
		"SELECT * FROM atoms WHERE type=8;" "outgoing"))

;(display "the duplicate pair list is: ")
;(display duplicate-pair-list) (newline)

;(define duplicate-eval-list
;	(look-for-dupes
;		"SELECT * FROM atoms WHERE type=47;" "outgoing"))
;
;(display "the duplicate eval list is: ")
;(display duplicate-eval-list) (newline)

; ------------------------------------------------

(define (make-outgoing-str uuid-list)
"
  make-outgoing-str -- Make a string that corresponds to the UUID list

  The string is used for forming SQL queries. It has the format, for
  example, '{123, 456}'
"
	(define ostr "'{")
	(set! ostr (string-concatenate
		(cons ostr (map
			; create list of comma-separated ints
			(lambda (x) (string-append (number->string x) ", "))
			 uuid-list))))
	; Wipe out the trailing comma
	(set! ostr (string-drop-right ostr 2))
	(set! ostr (string-append ostr "}'"))

	ostr
)

; ------------------------------------------------

(define (flush-query)
	(define row #f)
	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))
		(set! row (dbi-get_row conxion))
	)
)

; ------------------------------------------------
(define (delete-atoms uuid-list except)
"
  delete-atoms -- delete every atom in the uuid-list, except for
  the except uuid
"

	(define (del-atom uuid)
		(if (not (eq? uuid except))
			(begin
				(set! qry (string-append
					"DELETE FROM atoms WHERE uuid="
					(number->string uuid)))

				(display "Delete ")(display qry)(newline)
				(dbi-query conxion qry)
				(display (dbi-get_status conxion)) (newline)
				(flush-query)
			)
		)
	)
	; Loop over the list of uuids
	(for-each del-atom uuid-list)
)

(define (undup-eval luid-list)
"
  undup-eval -- consolidate duplicate EvaluationLinks

  The luid-list should be a list of integer uuids for the ListLinks
  that are the duplicates. The proceedure here is semi-manual;
  the total count is computed, but you have to update the count
  yourself, and also do the deletions.
"
	(define smallest-evid 2012123123)
	(define smallest-luid 2012123123)
	(define eval-list (list))
	(define count_tot 0)

	(define (sum-count uuid)
		(define row #f)
		(define qry "")
		; type=47 is the type of EvaluationLink
		(set! qry (string-append
			"SELECT * FROM atoms WHERE type=47 and outgoing="
			(make-outgoing-str (list uuid-of-any uuid)))
		)
		(display "Eval qry is ")(display qry) (newline)
		(dbi-query conxion qry)

		; Loop over table rows. There should be just one, I guess...
		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))

			; Extract the count of the EvalLink
			(let (
				(cnt (cdr (assoc "stv_count" row)))
				(eid (cdr (assoc "uuid" row))))

				(display "EvalLink uuid= ") (display eid)
				(display " cnt= ") (display cnt) (newline)
				(set! eval-list (cons eid eval-list))

				; Record the smallest UUID
				(if (< uuid smallest-luid)
					(begin
						(set! smallest-luid uuid)
						(set! smallest-evid eid)))

				; Sum the total count.
				(set! count_tot (+ count_tot cnt))
				(set! row (dbi-get_row conxion))
			)
		)
	)

	; Loop over the ListLink's
	(for-each sum-count luid-list)

	(display "total stv= ") (display count_tot) (newline)
	(display "uuid= ") (display smallest-evid) (newline)

	(set! qry (string-concatenate (list
		"UPDATE atoms SET stv_count="
		(number->string count_tot)
		" WHERE uuid="
		(number->string smallest-evid))))
	(display qry) (newline)
	(dbi-query conxion qry)
	(display (dbi-get_status conxion)) (newline)
	(flush-query)

	(delete-atoms eval-list smallest-evid)
	(delete-atoms luid-list smallest-luid)

	; Do this manually...
	; UPDATE atoms SET stv_count=5938 WHERE uuid=53650;
	; DELETE FROM atoms WHERE uuid=430743;
	; DELETE FROM atoms WHERE uuid=430742;
	;
	; UPDATE atoms SET stv_count=3760.0 WHERE uuid=27878;
	; DELETE FROM atoms WHERE uuid=415559;
	; DELETE FROM atoms WHERE uuid=415560;
)

(define (undup-pair pair)
"
  undup-pair Given a pair of UUID's that define a ListLink (well, more
  than one -- several duplicates), consolidate all of the duplicates.

  This works by obtaining a list of all of the duplicate UUID's, and
  then calling 'undup-eval' to consolidate them. The undup-eval
  sums up the counts, and deletes teh duplicates.
"

	(define row #f)
	(define uuid-list (list))

	; type=8 is the ListLink
	(define qry (string-append
		"SELECT * FROM atoms WHERE type=8 and outgoing="
		(make-outgoing-str pair)))

	(dbi-query conxion qry)

	(display "Duplicate list search status: ")
	(display (dbi-get_status conxion)) (newline)

	; Loop over table rows
	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the uuid of the ListLink
		(let ((uuid (cdr (assoc "uuid" row))))

			(display "ListLink uuid= ") (display uuid) (newline)
			(set! uuid-list (cons uuid uuid-list))
			(set! row (dbi-get_row conxion))
		)
	)

	; Consolidate duplicate EvaluationLinks
	(undup-eval uuid-list)
)

; (undup-pair (list 6709 137))
; (undup-pair (list 24493 101))

#! /usr/bin/env guile
!#
;
; Atomspace deduplication repair script
;
; Due to bugs, the SQL backend can end up with multiple copies of
; atoms. This script will find them, merge them, and sum the counts
; on the associated count truth values.  Its up to you to recompute
; anything else.

(use-modules (dbi dbi))
(use-modules (srfi srfi-1))


(define conxion
       ; (dbi-open "postgresql" "linas:asdf:en_pairs:tcp:localhost:5432"))
       (dbi-open "postgresql" "linas:asdf:en_pairs:socket:/var/run/postgresql"))

(display conxion) (newline)

; --------------------------------------------------------------
(define (look-for-dupes query colm)
"
  look-dor-dupes -- Look for duplicate atoms
  query should be the SQL query to perform
  colm should be the string column name; either 'name' or 'outgoing'

  returns a list of the duplicate entries found for 'colm'
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

(define duplicate-word-list
	(look-for-dupes
		"SELECT * FROM atoms WHERE type=73;" "name"))

;(display "the duplicate word list is: ")
;(display duplicate-word-list) (newline)
;
;(define duplicate-pair-list
;	(look-for-dupes
;		"SELECT * FROM atoms WHERE type=8;" "outgoing"))
;
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

(define (undup-eval uuid-list)
"
  undup-eval -- consolidte duplicate EvaluationLinks

  The uuid-list should be a list of integer uuids for the ListLinks
  that are the duplicates. The proceedure here is semi-manual;
  the total count is computed, but you have to update the count
  yourself, and also do the deletions.
"
	(define smallest-uuid 2012123123)
	(define row #f)
	(define count_tot 0)
	(define qry "")
	(define uuid 0)
	; Loop over the ListLink's
	(set! uuid (car uuid-list))
	(set! uuid-list (cdr uuid-list))

	(while (not (equal? uuid 0))


		(set! qry (string-append
			"SELECT * FROM atoms WHERE type=47 and outgoing="
			(make-outgoing-str (list 250 uuid)))
		)
		(display "Eval qry is ")(display qry) (newline)
		(dbi-query conxion qry)

		; Loop over table rows
		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))

			; Extract the count of the EvalLink
			(let (
				(cnt (cdr (assoc "stv_count" row)))
				(eid (cdr (assoc "uuid" row))))

				(display "EvalLink uuid= ") (display eid)
				(display " cnt= ") (display cnt) (newline)

				; Record the smallest UUID
				(if (< eid smallest-uuid) (set! smallest-uuid eid))

				; Sum the total count.
				(set! count_tot (+ count_tot cnt))
				(set! row (dbi-get_row conxion))
			)
		)

		; Get the next uuid on the list
		(if (not (equal? uuid-list (list)))
			(begin
				(set! uuid (car uuid-list))
				(set! uuid-list (cdr uuid-list))
			)
			(set! uuid 0)
		)
	)

	(display "total stv= ") (display count_tot) (newline)
	(display "uuid= ") (display smallest-uuid) (newline)

	(set! qry (string-concatenate (list
		"UPDATE atoms SET stv_count="
		(number->string count_tot)
		" WHERE uuid="
		(number->string smallest-uuid))))
	(display qry) (newline)

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
  undup-pair Given a pair of duplicate ListLinks, consolidate the two.

  Not only must we clobbr the list links, but also the EvaluationLinks
  that contain them.
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

; ------------------------------------------------
(define (find-pairs wuid)
"
  find-pairs -- given a uuid of single word, find all word-pairs
  which contain the word.
"
	(define row #f)

	; type=8 is the ListLink
	(define qry "SELECT * FROM atoms WHERE type=8")
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the outgoing set of the ListLink
		(let ((outset (cdr (assoc "outgoing" row))))
			(if (any (lambda (x) (eq? x wuid)) outset)
				(begin (display "contains ")(display outset)(newline))
			)
			(set! row (dbi-get_row conxion))
		)
	)
)
(find-pairs 6844)
; (find-pairs 27942)

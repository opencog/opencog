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
		(let ((word (cdr (assoc colm row))))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; Have we seen this item previously?
			(if (hash-ref word-set word)
				(begin
					(display "Oh no! Duplicate!! ") (display word) (newline)
					(set! dupe-list (cons word dupe-list))
				)
				(hash-set! word-set word word)
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

(display "the duplicate word list is: ")
(display duplicate-word-list) (newline)

(define duplicate-pair-list
	(look-for-dupes
		"SELECT * FROM atoms WHERE type=8;" "outgoing"))

(display "the duplicate pair list is: ")
(display duplicate-pair-list) (newline)

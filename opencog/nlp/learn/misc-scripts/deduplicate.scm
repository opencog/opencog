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

; Look for duplicate WordNodes
; First, get all WordNodes
(dbi-query conxion
	; "SELECT * FROM atoms WHERE type=73 LIMIT 10;")
	"SELECT * FROM atoms WHERE type=73;")

(display "WordNode duplicate search connection status: ")
(display (dbi-get_status conxion)) (newline)

(define word-list (list))
(define word-count 0)
(define row #f)
(set! row (dbi-get_row conxion))
(while (not (equal? row #f))
	(let ((word (cdr (assoc "name" row))))
		(set! word-count (+ word-count 1))
		(if (member word word-list)
			(begin (display "oh no! a duplicate!! ") (display word) (newline))
			(set! word-list (cons word word-list))
		)
		; (display word) (newline)
		(set! row (dbi-get_row conxion))
	)
)

(display "Word count was ") (display word-count) (newline)

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

(dbi-query conxion
	"SELECT * FROM atoms WHERE type=47 and outgoing='{250, 27877}';")

(display conxion) (newline)

(define row #f)
(set! row (dbi-get_row conxion))
(display "ahh")
(display row) (newline)
(while (not (equal? row #f))
	(display "wtf")
	(display row) (newline)
	(set! row (dbi-get_row conxion))
)

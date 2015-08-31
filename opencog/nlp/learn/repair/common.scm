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
; common code

(use-modules (dbi dbi))
(use-modules (srfi srfi-1))
(use-modules (rnrs io ports))

; debugging ...
(define do-update #t)

; fix up type codes
; (define EvalLinkType "47")
(define EvalLinkType (number->string 27))

(define WordNodeType (number->string 45))

; The uuid of the ANY LG type.  That is, the uuid of the
; LinkGrammarRelationshipNode "ANY"
; (define uuid-of-any 152)
(define uuid-of-any 57)

(define conxion
       ; (dbi-open "postgresql" "linas:asdf:en_pairs:tcp:localhost:5432"))
       ; (dbi-open "postgresql" "linas:asdf:en_pairs:socket:/var/run/postgresql"))
       (dbi-open "postgresql" "rohit:asdf:simple_pairs:tcp:localhost:5555"))

(display conxion) (newline)

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
					;(display "Oh no! Duplicate!! ") (display word) 
					;(display " prid=") (display wuid)
					;(display " uuid=") (display uuid)
					;(newline)
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

; ------------------------------------------------
(define* (delete-atoms uuid-list except #:optional do-prt)
"
  delete-atoms -- delete every atom in the uuid-list, except for
  the except uuid
"

	(define (del-atom uuid)
		(define qry "")
		(if (not (eq? uuid except))
			(begin
				(set! qry (string-append
					"DELETE FROM atoms WHERE uuid="
					(number->string uuid)))

				(if do-prt (begin
					(display "Delete ")(display qry)(newline)))
				(if do-update
					(begin
						(dbi-query conxion qry)
						(if do-prt (begin
							(display (dbi-get_status conxion)) (newline)))
						(flush-query)))
			)
		)
	)
	; Loop over the list of uuids
	(for-each del-atom uuid-list)
)

; --------------------------------------------------------------

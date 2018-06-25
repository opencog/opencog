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
; This script checks for orphaned UUID's

(load "common.scm")

; --------------------------------------------------------------

(define (get-atoms-of-height height)
"
  get-atms-of-height -- gt all atoms of a given height
"
	(define row #f)
	(define uuid 0)
	(define atom-list (list))

	; Find all the EvaluationLinks
	(define qry (string-append
		"SELECT * FROM atoms WHERE height="
		(number->string height)))
	(display qry)(newline)
	(dbi-query conxion qry)

	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		(set! uuid (cdr (assoc "uuid" row)))
		(set! atom-list (cons uuid atom-list))

		; Get the next row
		(set! row (dbi-get_row conxion))
	)
	atom-list
)

; --------------------------------------------------------------

(define (look-for-orphans uuid-list)
"
  look-for-orphans -- look for osets that don't have atoms

  Given a list of uuid's that are links, fetch the outgoing set
  of each link, and then look to see if the atoms in the outgoing
  set are in the database.  Return a list of the links that have
  outgoing sets that fail to refer to atoms.
"
	(define orphan-list (list))

	; Given a single uuid, return #t if it is in the atomspace,
	; else retun #f
	(define (check-atom uuid)
		(define row #f)
		(define have #f)

		; Find the oset
		(define qry (string-append
			"SELECT * FROM atoms WHERE uuid="
			(number->string uuid)))
		; (display qry)(newline)
		(dbi-query conxion qry)

		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))
			(set! have #t)
			; Get the next row
			(set! row (dbi-get_row conxion))
		)
		(if (not have) (begin
			(display "Cannot find uuid ")(display uuid)(newline))
		)
		have
	)

	; Given a single uuid, look at its oset.
	(define (check-oset uuid)
		(define row #f)
		(define oset-list (list))

		; Find the oset
		(define qry (string-append
			"SELECT outgoing FROM atoms WHERE uuid="
			(number->string uuid)))
		; (display qry)(newline)
		(dbi-query conxion qry)

		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))
			(set! oset-list (cdr (assoc "outgoing" row)))
			; Get the next row
			(set! row (dbi-get_row conxion))
		)

		; now check the oset
		(if (any (lambda (x) (not x)) (map check-atom oset-list))
			(begin
				(display "oh nooo!! bad link is ")(display uuid)(newline)
				(set! orphan-list (cons uuid orphan-list))
			)
		)
	)

	(for-each check-oset uuid-list)
	orphan-list
)

; --------------------------------------------------------------

(define h3 (get-atoms-of-height 3))
(display "Number of atoms of height 3: ") (display (length h3))(newline)

(define h2 (get-atoms-of-height 2))
(display "Number of atoms of height 2: ") (display (length h2))(newline)

(define o2 (look-for-orphans h2))
(display "Orphan h2 links: ") (display (length o2))(newline)

; (delete-atoms o2 0)

(define h1 (get-atoms-of-height 1))
(display "Number of atoms of height 1: ") (display (length h1))(newline)

(define o1 (look-for-orphans h1))
(display "Orphan h1 links: ") (display (length o1))(newline)

; (delete-atoms o1 0)

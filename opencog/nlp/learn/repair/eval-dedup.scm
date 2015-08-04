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
; This script focuses on duplicated EvaluationLinks.

(load "common.scm")

; --------------------------------------------------------------

; Look for duplicated EvaluationLinks.
; The list will consist of a pair of UUID's that identify the
; duplicate EvaluationLinks
(define duplicate-eval-list
	(look-for-dupes
		(string-concatenate
		"SELECT * FROM atoms WHERE type=" EvalLinkType ";") "outgoing"))

(display "The duplicate eval list has: ")
(display (length duplicate-eval-list)) (newline)


(define (eliminate-eval-dupes oset-list)
"
  eliminate-eval-dupes -- look for identical EvaluationLinks

  Given a list of outgoing-sets, sum the count stv, and update
  the count on one of the dupes, and delete the other dupe.
"
	; Sum the counts on the EvalLink
	(define (sum-counts oset)
		(define row #f)
		(define sum 0)
		(define uuid 0)
		(define smallest-uuid 2012123123)
		(define dup-list (list))

		; Find all the EvaluationLinks
		(define qry (string-append
			"SELECT * FROM atoms WHERE type="
			EvalLinkType
			" AND outgoing="
			(make-outgoing-str oset)))
		; (display qry)(newline)
		(dbi-query conxion qry)

		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))
			; Sum up the count values
			(set! sum (+ sum (cdr (assoc "stv_count" row))))

			; Find the smallest uuid.
			(set! uuid (cdr (assoc "uuid" row)))
			(if (< uuid smallest-uuid)
				(set! smallest-uuid uuid))

			(set! dup-list (cons uuid dup-list))

			; Get the next row
			(set! row (dbi-get_row conxion))
		)
		(let ((upd (string-append
				"UPDATE atoms SET stv_count="
				(number->string sum)
				" WHERE uuid="
				(number->string smallest-uuid)
				";")))
			(display upd) (newline)
			(if do-update (begin
				(dbi-query conxion upd)
				(display (dbi-get_status conxion)) (newline)
				(flush-query)))
			(delete-atoms dup-list  smallest-uuid)
		)
		smallest-uuid
	)

	(define cnt-list (map sum-counts oset-list))

	(display "oset: ") (display (length oset-list))(newline)
	(display "cnts: ") (display (length cnt-list))(newline)
)

(eliminate-eval-dupes duplicate-eval-list)

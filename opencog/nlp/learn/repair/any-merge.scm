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
; This script focuses on accidentally having two ANY nodes.
; Viz if select * from atoms where type=89 and name='ANY';
; returns more than one row :-(

(load "common.scm")

; --------------------------------------------------------------
(define (get-all-atoms query colm)
"
  get-all-atoms -- Execute the query, return all colm values.
  colm should be the string column name; e.g. 'uuid'

  Returns a list of the 'colm' entries
"
	(define alist (list))
	(define word-count 0)
	(define row #f)

	(dbi-query conxion query)

	(display "Atom search connection status: ")
	(display (dbi-get_status conxion)) (newline)

	; Loop over table rows
	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the column value
		(let* ((valu (cdr (assoc colm row))))
			(set! alist (cons valu alist))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; (display word) (newline)
			(set! row (dbi-get_row conxion))
		)
	)

	(display "Count was ") (display word-count) (newline)
	alist
)

; --------------------------------------------------------------
(define (get-all-evals alist anyid)
"
  get-all-evals -- Get all of the EvaluationLinks that contain
  a ListLink and the anyid uuid.

  Returns a list of the EvaluationLink entries.
"
	(define word-count 0)
	(define elist (list))

	; Get an evaluationLink
	(define (get-eval uuid)
		(define euid 0)
		(define row #f)
		(define qry (string-concatenate (list
			"SELECT uuid FROM atoms WHERE type="
			EvalLinkType
			" AND outgoing="
			(make-outgoing-str (list anyid uuid)))))
		; (display qry)(newline)
		(dbi-query conxion qry)

		; Loop over table rows
		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))

			; Extract the column value
			(set! euid (cdr (assoc "uuid" row)))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; (display word) (newline)
			(set! row (dbi-get_row conxion))
		)
		euid
	)

	(set! elist (map get-eval alist))
	(display "Count was ") (display word-count) (newline)
	elist
)

; --------------------------------------------------------------

; A list of all ListLinks
(define all-list-links (get-all-atoms
	"SELECT uuid FROM atoms WHERE type=8" "uuid"))

(display "Numb of 250-evals: ")
(display (length (get-all-evals all-list-links 250)))(newline)
(display "Numb of 152-evals: ")
(display (length (get-all-evals all-list-links 152)))(newline)

; --------------------------------------------------------------
(define (relabel-evals alist bad-id good-id)
"
  relabel-evals -- Change the oset of all of the EvaluationLinks
  that use the bad ANY uuid; make it use the good ANY id.

  Returns a list of the changed EvaluationLink entries.
"
	(define word-count 0)
	(define elist (list))

	; Change an EvaluationLink
	; euid == uuid of the evaluation link
	; luid == uuid of the ListLink
	; any-id == uuid to change it to.
	(define (set-eval euid luid any-id)
		(define row #f)
		(define qry (string-concatenate (list
			"UPDATE atoms SET outgoing="
			(make-outgoing-str (list any-id luid))
			" WHERE uuid="
			(number->string euid))))
		; (display qry)(newline)
		(dbi-query conxion qry)
		(flush-query)
	)

	; Change an EvaluationLink from the old bad ANY uuid to the new one
	; argument is the uuid of the ListLink
	(define (change-eval uuid)
		(define euid 0)
		(define row #f)
		(define qry (string-concatenate (list
			"SELECT uuid FROM atoms WHERE type="
			EvalLinkType
			" AND outgoing="
			(make-outgoing-str (list bad-id uuid)))))
		; (display qry)(newline)
		(dbi-query conxion qry)

		; Loop over table rows
		(set! row (dbi-get_row conxion))
		(while (not (equal? row #f))

			; Extract the column value
			(set! euid (cdr (assoc "uuid" row)))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; (display word) (newline)
			(set! row (dbi-get_row conxion))
		)
		(if (< 0 euid)
			(set-eval euid uuid good-id))
	)

	(set! elist (map change-eval alist))
	(display "Changed uuid count was ") (display word-count) (newline)
	elist
)

; (relabel-evals all-list-links 250 152)

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

	(display "For the query: ")(display query)(newline)
	(display "The num rows was: ") (display word-count) (newline)
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
	(display "Number of EvaluationLinks: ") (display word-count) (newline)
	elist
)

; --------------------------------------------------------------

; A list of all ListLinks
(define all-list-links (get-all-atoms
	"SELECT uuid FROM atoms WHERE type=8" "uuid"))

(define (count-evlinks any-uuid)
	(display "Numb of ") (display any-uuid) (display " evals: ")
	(display (length (get-all-evals all-list-links any-uuid)))(newline))

;(display "Numb of 250-evals: ")
;(display (length (get-all-evals all-list-links 250)))(newline)
;(display "Numb of 152-evals: ")
;(display (length (get-all-evals all-list-links 152)))(newline)

; (map count-evlinks (list 152 250))
; (map count-evlinks (list 57 139 140 186 190 270))


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

	; Change an EvaluationLink from the old bad ANY uuid to the new
	; one. The argument is the uuid of the ListLink
	(define (change-eval uuid)
		(define euid 0)
		(define row #f)

		; First, get the uuid of the EvaluationLink
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

			; Extract the uuid of the EvaluationLink
			(set! euid (cdr (assoc "uuid" row)))

			; Maintain a count, just for the hell of it.
			(set! word-count (+ word-count 1))

			; Print status so we don't get bored.
			(if (eq? 0 (modulo word-count 1000)) (begin
				(display "Processed ")(display word-count)
				(display " id-relabels")(newline))
				(flush-output-port (current-output-port))
			)

			; (display word) (newline)
			(set! row (dbi-get_row conxion))
		)

		; euid will be zero is the ListLink does not appear with
		; the bad any-id
		(if (< 0 euid)
			(set-eval euid uuid good-id))
	)

	(set! elist (map change-eval alist))
	(display "Relabel ANY uuid ") (display bad-id)
	(display " to ") (display good-id)(newline)
	(display "Relabeled uuid count was ") (display word-count) (newline)
	(flush-output-port (current-output-port))
	elist
)

; (relabel-evals all-list-links 250 152)

;(relabel-evals all-list-links 139 57)
;(relabel-evals all-list-links 140 57)
;(relabel-evals all-list-links 186 57)
;(relabel-evals all-list-links 190 57)
;(relabel-evals all-list-links 270 57)

; -----------------------------------------------

(define (get-all-non-any-evals any-id)
"
  get-all-non-any-evals -- look for all EvalLinks that do NOT
  hold the desired ANY node.  At this point in the game, there
  should not be any of these. But there are. WTF. Oh, it was
  a bad conversion of int8 to long long in guile-dbi.
"
	(define bad-list (list))
	(define euid 0)
	(define luid 0)
	(define oset (list))
	(define row #f)
	(define qry (string-concatenate (list
		"SELECT uuid,outgoing FROM atoms WHERE type="
		EvalLinkType)))
	(display qry)(newline)
	(dbi-query conxion qry)

	; Loop over table rows
	(set! row (dbi-get_row conxion))
	(while (not (equal? row #f))

		; Extract the column value
		(set! euid (cdr (assoc "uuid" row)))
		(set! oset (cdr (assoc "outgoing" row)))
		(set! luid (cadr oset))

		(if (not (eq? (car oset) any-id)) (begin
			(set! bad-list (cons luid bad-list))
			;(display "Its bad: ")(display euid)
			;(display " any: ")(display (car oset))(newline)
			;(flush-output-port (current-output-port))
		))

		; (display word) (newline)
		(set! row (dbi-get_row conxion))
	)

	; Return the list of bad EvaluationLinks
	bad-list
)

(define bad-list (get-all-non-any-evals 57))
(display "Number of bad evals: ") (display (length bad-list))(newline)

;(relabel-evals bad-list 139 57)
;(relabel-evals bad-list 140 57)
;(relabel-evals bad-list 186 57)
;(relabel-evals bad-list 190 57)
;(relabel-evals bad-list 270 57)

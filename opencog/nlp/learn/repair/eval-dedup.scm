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

; look for duplicated EvaluationLinks
(define duplicate-eval-list
	(look-for-dupes
		"SELECT * FROM atoms WHERE type=47;" "outgoing"))

(display "The duplicate eval list has: ")
(display (length duplicate-eval-list)) (newline)


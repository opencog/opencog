scm
;
; file-process.scm
; Work through a bunch of RelEx files that are in opencog format,
; import each into the opencog server, run the triples code on each,
; thencleanup, and do the next file.
;
; Linas Vepstas April 2009
;
(use-modules (ice-9 rdelim))
(use-modules (ice-9 popen))

; ---------------------------------------------------------------------
;
(define (process-data input-dir done-dir num-to-do)

	(define (process-file filename)
	)
)

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
		(let ((fullname (string-join (list input-dir filename) "/"))
				(donename (string-join (list done-dir filename) "/"))
			)
			(system "date")
			(system (string-join (list "echo will load: \"" filename "\"")))

			(load-scm-from-file fullname)
			(fire-all-triple-rules) 

			(system (string-join (list "mv \"" fullname "\" \"" donename "\"") ""))

			(delete-sentences)
			(system (string-join (list "echo done delete: \"" filename "\"")))
		)
	)

	(for-each process-file
		(take (list-files input-dir) num-to-do)
	)
)

(define in-dir "/home/linas/src/novamente/data/conceptnet/parsed")
(define done-dir "/home/linas/src/novamente/data/conceptnet/tripled")
(define (do-triples n) (process-data in-dir done-dir n))

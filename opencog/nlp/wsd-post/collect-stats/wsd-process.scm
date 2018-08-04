;
; wsd-process.scm
; Work through a bunch of RelEx files that are in the compact-file-format,
; and import these into opencog. Then WSD run processing on each one, as
; needed. This is a batch-processing script.
;
; Copyright (C) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
(use-modules (ice-9 rdelim))
(use-modules (ice-9 popen))

; ---------------------------------------------------------------------
; Perform WSD-disjunct statistics processing.
;
; Process 'num-to-do' files in the input dir, and move them to done-dir
; The input files are assumed to be RelEx "compact-file format" files.
; These are converted into opencog data, and are loaded into opencog.
; At this point WSD should run automatically; this script then triggers
; disjunct processing manually. When this completes, the next file is
; processed ... 
;
(define (wsd-process-data input-dir done-dir num-to-do)
	(define cnt 0)

	(define (process-file filename)
		(let ((fullname (string-join (list input-dir filename) "/"))
		      (donename (string-join (list done-dir filename) "/"))
			)
			(system (string-join (list "echo \"" (string-join 
				(map (lambda (x) (string-join (list (object->string x) "\n"))) 
				(gc-stats))) "\"")))
			(system (string-join (list "echo \"" (string-join 
				(map (lambda (x) (string-join (list (object->string x) "\n"))) 
				(cog-report-counts))) "\"")))
			(system (string-join (list "echo start article "
				(object->string cnt)
				": \"" filename "\""))
			)
			(system "date")
			(load-cff-data fullname)
			(system (string-join (list "echo done cff: \"" filename "\"")))
			(system "date")
			(run-wsd)
			(system (string-join (list "echo done wsd: \"" filename "\"")))
			(system "date")
			(ldj-process)
			(system (string-join (list "mv \"" fullname "\" \"" donename "\"") ""))
			(system (string-join (list "echo done ldj: \"" filename "\"")))
			(system "date")
			(delete-sentences)
			(system (string-join (list "echo done delete: \"" filename "\"")))
			(system "date")
			(set! cnt (+ cnt 1))
			(system (string-join (list "echo done file: \"" (object->string cnt) "\"")))
		)
	)
	
	(for-each process-file
		(take (list-files input-dir) num-to-do)
	)
)

(define (do-wsd num-to-do) (wsd-process-data input-filedir done-filedir num-to-do))



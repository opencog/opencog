scm
;
; mit-conceptnet.scm
;
; Load and batch-process declarative "commonsense" sentences from 
; the MIT ConceptNet project. 
;
; Copyright (C) 2008, 2009 Linas Vepstas <linasvepstas@gmail.com>
;

(use-modules (ice-9 rdelim))
(use-modules (ice-9 popen))

; ---------------------------------------------------------------------
; Read in data from RelEx "compact file format" file, convert it to
; opencog format (using the perl script "cff-to-opencog.pl"), and
; then load it into opencog.
;
(define (load-cff-bz2-data filename)
	(exec-scm-from-cmd 
		(string-join (list "cat \"" filename "\" | bunzip2 | " cff-to-opencog-exe) "")
	)
)

; ---------------------------------------------------------------------
; Perform MIT ConceptNet concept extraction processing.
;
; Process 'num-to-do' files in the input dir, and move them to done-dir
; The input files are assumed to be RelEx "compact-file format" files.
; These are converted into opencog data, and are loaded into opencog.
; This script then runs triple processing, and seme extraction ...
;
(define (seme-process-data input-dir done-dir num-to-do)

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
			(system (string-join (list "echo start file: \"" filename "\"")))
			(system "date")
			(load-cff-bz2-data fullname)
			(system (string-join (list "echo done cff: \"" filename "\"")))
			(system "date")
			(system (string-join (list "mv \"" fullname "\" \"" donename "\"") ""))
		)
	)
	
	(for-each process-file
		(take (list-files input-dir) num-to-do)
	)
)

(define (do-seme num-to-do) (seme-process-data mcn-input-filedir mcn-done-filedir num-to-do))

.
exit


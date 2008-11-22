scm
;
; file-process.scm
; Work through a bunch of relex files that are in the compact-file-format,
; and import these into opencog. Then run processing on each one, as needed.
;
(define input-filedir "/home/linas/src/novamente/data/enwiki/enwiki-20080524/parsed/E")
(define done-filedir "/home/linas/src/novamente/data/enwiki/enwiki-20080524/coged/E")

(define cff-to-opencog-exe "/home/linas/src/novamente/src/relex-bzr/src/perl/cff-to-opencog.pl")

(use-modules (ice-9 rdelim))
(use-modules (ice-9 popen))

; ---------------------------------------------------------------------
; Read in data from RelEx "compact file format" file, convert it to
; opencog format (using the perl script "cff-to-opencog.pl"), and
; then load it into opencog.
;
(define (load-cff-data filename)
	(exec-scm-from-cmd 
		(string-join (list "cat \"" filename "\" | " cff-to-opencog-exe) "")
	)
)

; ---------------------------------------------------------------------
; Process all the files in the input dir, and move them to donedir
; The input files are assumed to be RelEx "compact-file format" files.
; These are converted into opencog data, and are loaded into opencog.
; At this point wsd should run automatically; this script then triggers
; disjunct processing manually. When this completes, the next file is
; processed ... 
;
(define (process-data input-dir done-dir)

	(define (process-file filename)
		(let ((fullname (string-join (list input-dir filename) "/"))
		      (donename (string-join (list done-dir filename) "/"))
			)
			(system (string-join (list "echo start article: \"" filename "\"")))
			(system "date")
			; (display "Starting: ") 
			; (display filename) (newline)
			(load-cff-data fullname)
			(system (string-join (list "echo done cff: \"" filename "\"")))
			(system "date")
			(cog-ad-hoc "do-wsd")
			(system (string-join (list "echo done wsd: \"" filename "\"")))
			(system "date")
			(ldj-process)
			(system (string-join (list "echo done ldj: \"" filename "\"")))
			(system "date")
			(system (string-join (list "mv \"" fullname "\" \"" donename "\"") ""))
		)
	)
	
	(for-each process-file
		(list-files input-dir)
	)
)

(define (doit) (process-data input-filedir done-filedir))

.
exit


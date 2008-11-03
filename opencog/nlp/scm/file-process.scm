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
; Given a directory, return a list of all of the files in the directory
; Do not return anything that is a subdirectory, pipe, special file etc.
;
(define (listfiles dir)

	(define (isfile? file)
		(if (eq? 'regular (stat:type (stat 
				(string-join (list dir file) "/"))))
			#t
			#f
		)
	)

	; suck all the filenames off a port
	(define (suck-in-filenames port lst)
		(let ((one-file (readdir port)))
			(if (eof-object? one-file)
				lst
				(suck-in-filenames port 
					(if (isfile? one-file)
						(cons one-file lst)
						lst
					)
				)
			)
		)
	)
	(let* ((dirport (opendir dir))
			(filelist (suck-in-filenames dirport '()))
		)
		(closedir dirport)
		filelist
	)
)

; ---------------------------------------------------------------------
; Read in data from RelEx "compact file format" file, convert it to
; opencog format (using the perl script "cff-to-opencog.pl"), and
; then load it into opencog.
;
(define (load-cff-data filename)

	; Suck in a bunch of ASCII text off of a port, until the port is
	; empty (#eof) and return a string holding the port (file) contents.
	(define (suck-in-text port str)
		(let ((one-line (read-line port)))
			(if (eof-object? one-line)
				str
				(suck-in-text port 
					(string-join (list str one-line "\n"))
				)
			)
		)
	)

	(let* ((cmd (string-join (list "cat" filename " | " cff-to-opencog-exe)))
			(port (open-input-pipe cmd))
			(data (suck-in-text port ""))
		)
		; (read port)
		(eval-string data)
		(close-pipe port)
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
			(display "Starting: ") 
			(display filename) (newline)
			(load-cff-data fullname)
			(ldj-process)
			(system (string-join (list "mv" fullname donename)))
		)
	)
	
	(for-each process-file
		(listfiles input-dir)
	)
)

(define (doit) (process-data input-filedir done-filedir))

.
exit


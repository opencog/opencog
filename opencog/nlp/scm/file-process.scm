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

(define (cvt-file filename)

	(define (suck-in-data port str)
		(let ((one-line (read-line port)))
			(if (eof-object? one-line)
				str
				(suck-in-data port 
					(string-join (list str one-line "\n"))
				)
			)
		)
	)

	; (cmd (string-join (list "cat" filename " | " cff-to-opencog-exe)))
	(let* ((cmd (string-join (list "cat" filename)))
			(port (open-input-pipe cmd))
		)
(display
		(suck-in-data port "")
)
		(close-pipe port)
	)
)

(define (process-data)

)

.
exit


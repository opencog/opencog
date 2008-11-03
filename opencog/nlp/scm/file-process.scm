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
		(close-pipe port)
		(eval-string data)
	)
)

(define (process-data)

)

.
exit


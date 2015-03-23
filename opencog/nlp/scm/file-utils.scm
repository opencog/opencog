;
; file-utils.scm
;
; Assorted file utilities.
;
; Copyright (C) 2008, 2009 Linas Vepstas <linasvepstas@gmail.com>
;

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
; Read in data from a bzip2-compressed RelEx "compact file format" file,
; convert it to ; opencog format (using the perl script "cff-to-opencog.pl"),
; and then load it into opencog.
;
(define (load-cff-bz2-data filename)
	(exec-scm-from-cmd 
		(string-join (list "cat \"" filename "\" | bunzip2 | " cff-to-opencog-exe) "")
	)
)


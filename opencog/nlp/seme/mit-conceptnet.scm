;
; mit-conceptnet.scm
;
; Load and process declarative "commonsense" sentences from the MIT
; ConceptNet project. 
;
; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;

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


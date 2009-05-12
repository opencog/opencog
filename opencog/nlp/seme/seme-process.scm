scm
;
; seme-process.scm
;
; Perform seme processing
;
; Copyright (C) 2009 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
; 

; do-seme-processing -- ad-hoc routine under development.
;
(define (do-seme-processing)

	(for-each (lambda (x) (display "duude got a sent\n") (display x))
		(get-new-parsed-sentences)
	)

)

.
exit

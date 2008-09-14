scm
;
; utilities.scm
;
; Miscellaneous handy utilities.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;

; -----------------------------------------------------------------------
; cog-filter atom-type proc atom-list
;
; Apply the proceedure proc to every atom of atom-list that is
; of type atom-type. Application halts if proc returns any value 
; other than #f
;
; Exmaple usage:
; (cog-filter 'ConceptNode display (list (cog-new-node 'ConceptNode "hello")))
; 
(define (cog-filter atom-type proc atom-list) 
	(define rc #f)
	(cond 
		((null? atom-list) #f)
		((eq? (cog-type (car atom-list)) atom-type) 
			(set! rc (proc (car atom-list))) 
			(if (eq? #f rc) 
				(cog-filter atom-type proc (cdr atom-list)))
		) 
		(else (cog-filter atom-type proc (cdr atom-list))
		)
	)
	rc
)


; exit scheme shell, exit opencog shell.
.
exit

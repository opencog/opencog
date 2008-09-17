scm
;
; utilities.scm
;
; Miscellaneous handy utilities.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
;
(define (stv mean conf) (cog-new-stv mean conf))

; -----------------------------------------------------------------------
; cog-filter atom-type proc atom-list
;
; Apply the proceedure 'proc' to every atom of 'atom-list' that is
; of type 'atom-type'. Application halts if proc returns any value 
; other than #f. Return the last value returned by proc; that is,
; return #f if proc always returned #f, otherwise return the value
; that halted the application.
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

; -----------------------------------------------------------------------
;
; cog-map-chase-link link-type endpoint-type dbg-lmsg dbg-emsg proc anchor
;
; Chase 'link-type' to 'endpoint-type' and apply proc to what is found there.
;
; It is presumed that 'anchor' points to some atom (typically a node),
; and that it has many links in its incoming set. So, loop over all of
; the links of 'link-type' in this set. They presumably link to all 
; sorts of things. Find all of the things that are of 'endpoint-type'
; and then call 'proc' on each of these endpoints. Optionally, print
; some debugging msgs.
;
; The link-chasing halts if proc returns any value other than #f.
; Returns the last value returned by proc, i.e. #f, or the value that
; halted the iteration.
;
; Example usage:
; (cog-map-chase-link 'ReferenceLink 'WordNode "" "" proc word-inst)
; Given a 'word-inst', this will chase all ReferenceLink's to all 
; WordNode's, and then will call 'proc' on these WordNodes.
;
(define (cog-map-chase-link link-type endpoint-type dbg-lmsg dbg-emsg proc anchor)
	(define (get-endpoint w)
		(display dbg-emsg)
		; cog-filter returns the return value from proc, we pass it on
		; in turn, so make sure this is last statement
		(cog-filter endpoint-type proc (cog-outgoing-set w))
	)
	(display dbg-lmsg)
	; cog-filter returns the return value from proc, we pass it on
	; in turn, so make sure this is last statement
	(cog-filter link-type get-endpoint (cog-incoming-set anchor))
)

; -----------------------------------------------------------------------
; exit scheme shell, exit opencog shell.
.
exit

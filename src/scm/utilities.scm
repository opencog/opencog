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
; Apply the proceedure 'proc' to every atom of 'atom-list' that is
; of type 'atom-type'. Application halts if proc returns any value 
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
; The link-chasing halts if proc returns any value other than #f
;
; Example usage:
; (cog-map-chase-link 'ReferenceLink 'WordNode "" "" proc word-inst)
; Given a 'word-inst', this will chase all ReferenceLink's to all 
; WordNode's, and then will call 'proc' on these WordNodes.
;
(define (cog-map-chase-link link-type endpoint-type dbg-lmsg dbg-emsg proc anchor)
	(define rc #f)
	(define (get-endpoint w)
		(set! rc (cog-filter endpoint-type proc (cog-outgoing-set w)))
		(display dbg-emsg)
		rc
	)
	(set! rc (cog-filter link-type get-endpoint (cog-incoming-set anchor)))
	(display dbg-lmsg)
	rc
)

; -----------------------------------------------------------------------
; exit scheme shell, exit opencog shell.
.
exit

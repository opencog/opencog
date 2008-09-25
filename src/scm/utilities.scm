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
; analogs of car, cdr, etc. but for atoms.
(define (gar x) (if (cog-atom? x) (car (cog-outgoing-set x)) (car x)))
(define (gdr x) (if (cog-atom? x) (cdr (cog-outgoing-set x)) (cdr x)))
(define (gadr x) (if (cog-atom? x) (cadr (cog-outgoing-set x)) (cadr x)))
(define (gaddr x) (if (cog-atom? x) (caddr (cog-outgoing-set x)) (caddr x)))

; A more agressive way of doing the above:
; (define car (let ((oldcar car)) (lambda (x) (if (cog-atom? x) (oldcar (cog-outgoing-set x)) (oldcar x)))))
; But this would probably lead to various painful debugging situations.

; -----------------------------------------------------------------------
; for-each-except 
; standard for-each loop, except that anything matchin "except" is skipped
(define (for-each-except exclude proc lst)
	(define (loop items)
		(cond
			((null? items) #f)
			((eq? exclude (car items))
				(loop (cdr items))
			)
			(else
				(proc (car items))
				(loop (cdr items))
			)
		)
	)
	(loop lst)
)

; -----------------------------------------------------------------------
;
; cog-get-atoms atom-type
; Return a list of all atoms in the atomspace that are of type 'atom-type'
;
; Example usage:
; (display (cog-get-atoms 'ConceptNode))
; will return and display all atoms of type 'ConceptNode
;
(define (cog-get-atoms atom-type)
	(let ((lst '()))
		(define (mklist atom)
			(set! lst (cons atom lst))
			#f
		)
		(cog-map-type mklist atom-type)
		lst
	)
)
;
; -----------------------------------------------------------------------
; cog-filter-for-each atom-type proc atom-list
;
; Apply the proceedure 'proc' to every atom of 'atom-list' that is
; of type 'atom-type'. Application halts if proc returns any value 
; other than #f. Return the last value returned by proc; that is,
; return #f if proc always returned #f, otherwise return the value
; that halted the application.
;
; Exmaple usage:
; (cog-filter-for-each 'ConceptNode display (list (cog-new-node 'ConceptNode "hello")))
; 
; See also: cgw-filter-atom-type, which does the same thing, but for wires.
;
(define (cog-filter-for-each atom-type proc atom-list) 
	(define rc #f)
	(cond 
		((null? atom-list) #f)
		((eq? (cog-type (car atom-list)) atom-type) 
			(set! rc (proc (car atom-list))) 
			(if (eq? #f rc) 
				(cog-filter-for-each atom-type proc (cdr atom-list))
				rc
			)
		) 
		(else (cog-filter-for-each atom-type proc (cdr atom-list))
		)
	)
)

(define (cog-filter atom-type atom-list) 
	(define (is-type? atom) (eq? atom-type (cog-type atom)))
	(filter is-type? atom-list)
)

(define (cog-filter-incoming atom-type atom)
	(cog-filter atom-type (cog-incoming-set atom))
)

; -----------------------------------------------------------------------
;
; cog-chase-link link-type endpoint-type anchor
;
; Starting at the atom 'anchor', chase its incoming links of
; 'link-type', and return a list of all of the 'node-type' in
; those links.
;
; It is presumed that 'anchor' points to some atom (typically a node),
; and that it has many links in its incoming set. So, loop over all of
; the links of 'link-type' in this set. They presumably link to all 
; sorts of things. Find all of the things that are of 'endpoint-type'.
; Return a list of all of these.
;
; See also: cgw-follow-link, which does the same thing, but for wires.
;
(define (cog-chase-link link-type endpoint-type anchor)
	(let ((lst '()))
		(define (mklist inst)
			(set! lst (cons inst lst))
			#f
		)
		(cog-map-chase-link link-type endpoint-type '() '() mklist anchor)
		lst
	)
)

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
		(if (not (eq? '() dbg-emsg)) (display dbg-emsg))
		; cog-filter-for-each returns the return value from proc, we pass it on
		; in turn, so make sure this is last statement
		(cog-filter-for-each endpoint-type proc (cog-outgoing-set w))
	)
	(if (not (eq? '() dbg-lmsg)) (display dbg-lmsg))
	; cog-filter-for-each returns the return value from proc, we pass it on
	; in turn, so make sure this is last statement
	(cog-filter-for-each link-type get-endpoint (cog-incoming-set anchor))
)

; -----------------------------------------------------------------------
; exit scheme shell, exit opencog shell.
.
exit

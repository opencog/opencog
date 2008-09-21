scm
;
; cog-stream.scm
;
; Streams and stream-processors of cog atoms. See wiring.scm for details.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;

; Place all atoms in the atomspace, of type 'atom-type', onto wire
(define (cgw-source-atoms wire atom-type)
	(wire-source-list wire (cog-get-atoms atom-type))
)


; Transform an atom to its incoming/outgoing list
; So if ...
(define (cgw-xfer up-wire down-wire)

	(define (up-me msg)
		(display "up duude: ")
		(display msg)
		(newline)
		(cond 
			((eq? msg wire-assert-msg)
			)
			
			((eq? msg wire-float-msg)
			)
			(else (error "Unknown message -- cgw-xfer" msg))
		)
	)
	(define (down-me msg)
		(display "down duude: ")
		(display msg)
		(newline)
	)

	(wire-connect up-wire up-me)
	(wire-connect down-wire down-me)

	'()
)


.
exit

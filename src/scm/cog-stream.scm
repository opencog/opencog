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

	(let ((up-state wire-float-msg)
			(down-state wire-float-msg)
		)

		(define (do-stuff x)
			; (wire-set-value! down-wire xxx down-me)
			'()
		)

		(define (up-me msg)
			(display "up duude: ")
			(display msg)
			(newline)
			(cond 
				((eq? msg wire-assert-msg)
					(if (eq? down-state wire-assert-msg)
						(error "Both inputs asserted - cgw-xfer up-wire" )
						(begin
							(set! up-state msg)
							;; XXX do something here this is wrong
							; (stream-for-each do-stuff (wire-get-stream wire))
						)
					)
				)
				
				((eq? msg wire-float-msg)
					(set! up-state msg)
					; XXX anything else here?
				)
				(else (error "Unknown message -- cgw-xfer up-wire"))
			)
		)
		(define (down-me msg)
			(display "down duude: ")
			(display msg)
			(newline)
			(cond
				((eq? msg wire-assert-msg)
					;; XXX do something here
				)
				((eq? msg wire-float-msg)
					; transmit on the down wire, if we have something to xmit
					(if (eq? up-state wire-assert-msg)
						(begin
							(display "got stuff on up, neet to send down\n")
							(wire-set-stream! down-wire (list->stream (list 'a 'b 'c)) down-me) ;; XX the wrong messge
						)
						; else XXX ?? 
					)
				)
				(else (error "Unknown message -- cgw-xfer down-wire"))
			)
		)

		(wire-connect up-wire up-me)
		(wire-connect down-wire down-me)
	)

	'()
)


.
exit

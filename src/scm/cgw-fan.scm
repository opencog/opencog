scm
;
; cgw-fan.scm
;
; Implements fanouts and fan-ins
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
;
; wire-fan-out a-wire b-wire c-wire
;
; Given a stream on any one of the wires, repeat it onto
; the other two.
;
(define (wire-fan-out a-wire b-wire c-wire)

	(let ((myname ""))
		; Take the input stream, pass it to the output streams.
		(define (fan in-wire a-out-wire b-out-wire)
			(let ((in-stream (wire-take-stream in-wire)))
				(wire-connect a-out-wire me)
				(wire-connect b-out-wire me)
				(wire-set-stream! a-out-wire in-stream me)
				(wire-set-stream! b-out-wire in-stream me)
			)
		)
	
		(define (process-msg)
			(cond
				((and 
						(wire-has-stream? a-wire)
						(not (wire-has-stream? b-wire))
						(not (wire-has-stream? c-wire))
					)
					(fan a-wire b-wire c-wire)
				)
				((and 
						(not (wire-has-stream? a-wire))
						(wire-has-stream? b-wire)
						(not (wire-has-stream? c-wire))
					)
					(fan b-wire c-wire a-wire)
				)
				((and 
						(not (wire-has-stream? a-wire))
						(not (wire-has-stream? b-wire))
						(wire-has-stream? c-wire)
					)
					(fan c-wire a-wire b-wire)
				)
			)
		)

		(define (me msg)
			(cond 
				((eq? msg wire-assert-msg)
					(process-msg)
				)
				((eq? msg wire-float-msg)
					;; do nothing
				)
				(else
					(default-dispatcher msg 'wire-fan-out myname)
				)
			)
		)

		(wire-connect a-wire me)
		(wire-connect b-wire me)
		(wire-connect c-wire me)
		me
	)
)

; --------------------------------------------------------------------
.
exit

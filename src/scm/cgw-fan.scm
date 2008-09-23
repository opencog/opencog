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
;
; wire-fan-in a-wire b-wire c-wire
;
; Given a stream on any two wires, repeat only those parts
; of the stream that agree with each-other on the third wire.
;
(define (wire-fan-in a-wire b-wire c-wire)
)


; --------------------------------------------------------------------
;
; wire-comparator a-wire b-wire c-wire
;
; Compare input streams on any two wires, presenting an output stream
; on the third wire. The function 'compare-func' is used to generate
; the output stream: it must take two elements as input, and produce
; a list of elements as output (or produce a null list).  This function
; is entirely analogous to wire-transciever, except that the transformation
; function takes two inputs.
;
(define (wire-comparator a-wire b-wire c-wire compare-func)

	(let ((myname "")
			(a-in-stream stream-null)
			(b-in-stream stream-null)
			)

		; Define a generic producer function for a pair of streams. This
		; producer pulls an element off of each input stream, and applies
		; the function 'xform-func' to produce an output. The 'xform-func'
		; should produce a list of elements; these are then posted.
		(define (producer state xform-func)
			; If we are here, we're being forced.
			(if (null? state)
				(if (or
						(stream-null? a-in-stream)
						(stream-null? b-in-stream)
					)
					#f ; we are done, one of the other stream has been drained dry
					; else grab an element from each input-stream
					(let ((a-atom (stream-car a-in-stream))
							(b-atom (stream-car b-in-stream)))
						(set! a-in-stream (stream-cdr a-in-stream))
						(set! b-in-stream (stream-cdr b-in-stream))
						(if (or (null? a-atom) (null? b-atom))
							(error "Unexpected empty stream! wire-fan-in")
							(producer (xform-func a-atom b-atom) xform-func)
						)
					)
				)
				; else state is a list of elements, so keep letting 'er rip.
				state
			)
		)

		; Compare two input streams, only allow matches to pass
		(define (fan-in a-in-wire b-in-wire out-wire)
			(set! a-in-steam (wire-take-stream a-in-wire))
			(set! b-in-steam (wire-take-stream b-in-wire))
		)
	
		(define (process-msg)
			(cond
				((and 
						(wire-has-stream? a-wire)
						(wire-has-stream? b-wire)
						(not (wire-has-stream? c-wire))
					)
					(fan-in a-wire b-wire c-wire)
				)
				((and 
						(wire-has-stream? a-wire)
						(not (wire-has-stream? b-wire))
						(wire-has-stream? c-wire)
					)
					(fan-in c-wire a-wire b-wire)
				)
				((and 
						(not (wire-has-stream? a-wire))
						(wire-has-stream? b-wire)
						(wire-has-stream? c-wire)
					)
					(fan-in b-wire c-wire a-wire)
				)
				((and 
						(wire-has-stream? a-wire)
						(wire-has-stream? b-wire)
						(wire-has-stream? c-wire)
					)
					(error "All three wires have streams! -- wire-fan-in")
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
					(default-dispatcher msg 'wire-fan myname)
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

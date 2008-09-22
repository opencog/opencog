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
			(input-stream stream-null)
		)

		; Define a producer function for a stream. This producer pulls
		; atoms off the input stream, and posts the incoming set of 
		; each atom.
		(define (get-incoming state)
			(producer state cog-incoming-set)
		)

		; Define a producer function for a stream. This producer pulls
		; atoms off the input stream, and posts the outgoing set of 
		; each atom.
		(define (get-outgoing state)
			(producer state cog-outgoing-set)
		)

		; Define a generic producer function for a stream. This producer 
		; pulls atoms off the input stream, and applies the function 
		; 'cog-func'. This function should produce a list of atoms; these
		; atoms are then posted
		(define (producer state cog-func)
			; If we are here, we're being forced.
			(if (null? state)
				(if (stream-null? input-stream)
					#f ; we are done, the stream has been drained dry
					; else grab an atom from the input-stream
					(let ((atom (stream-car input-stream)))
						(set! input-stream (stream-cdr input-stream))
						(if (null? atom)
							(error "Unexpected empty stream! cgw-xfer up-wire")
							(cog-func atom) ;; spool out the (incoming or outgoing) set
							; XXX need to handle the case of empty list!
						)
					)
				)
				; else state is a list of atoms, so keep letting 'er rip.
				state
			)
		)

		; Pull atoms off the up-stream, get thier incoming set, and post
		; the incoming set to the down-stream.
		(define (make-down-stream)
			(if (not (wire-has-stream? up-wire))
				(error "Impossible condition: up-wire has no stream! -- cgw-xfer")
			)
			(set! input-stream (wire-take-stream up-wire))
			(if (stream-null? input-stream)
				(error "input stream is unexpectedly empty - cgw-xfer")
			)
			; (list->stream (list 'a 'b 'c))  ; sample test gen
			(make-stream get-incoming '())
		)

		; Pull atoms off the down-stream, get thier outgoing set, and post
		; the outgoing set to the up-stream.
		(define (make-up-stream)
			(if (not (wire-has-stream? down-wire))
				(error "Impossible condition: down-wire has no stream! -- cgw-xfer")
			)
			(set! input-stream (wire-take-stream down-wire))
			(if (stream-null? input-stream)
				(error "input stream is unexpectedly empty - cgw-xfer")
			)
			; (list->stream (list 'a 'b 'c))  ; sample test gen
			(make-stream get-outgoing '())
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
							(set! up-state wire-float-msg) ;; XX hey ...
							; If we are here, there's a stream on the up-wire. 
							; transform it and send it.
	(display "too agot stuff on up, need to send down\n")
							(wire-set-stream! down-wire (make-down-stream) down-me)
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
					(set! down-state msg)
					; transmit on the down wire, if we have something to xmit
					(if (eq? up-state wire-assert-msg)
						(begin
							; If we are here, there's a stream on the up-wire. 
							; transform it and send it.
	(display "got stuff on up, need to send down\n")
							(wire-set-stream! down-wire (make-down-stream) down-me)
						)
						; else the up-wire state is floating, 
						; and we don't do anything here.
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

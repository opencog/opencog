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

		(define (get-incoming state)
			; If we are here, we're being forced.
			(if (null? state)
				(if (stream-null? input-stream)
(begin (display "no more atoms!\n")
					#f ; we are done, the stream has been drained dry
)
					; else grab an atom from the up-stream
					(let ((atom (stream-car input-stream)))
						(set! input-stream (stream-cdr input-stream))
						(if (null? atom)
							(error "Unexpected empty stream! cgw-xfer up-wire")
(begin (display "posting incoming set\n")
							(cog-incoming-set atom) ;; spool out the incoming-set
)
						)
					)
				)
				; else state is a list of atoms, so keep letting 'er rip.
(begin (display "rip one off\n")
				state
)
			)
		)

		(define (xget-incoming state)
			; If we are here, we're being forced.
			(if (null? state)
(begin
(display "duude nullo\n")
				(let ((atom (stream-car input-stream)))
(display "duude atomo is\n")
(display input-stream)
(newline)
(display (stream-null? input-stream))
(newline)
(display (stream-car input-stream))
(newline)
				)
)
			)
			#f
		)


		; Pull atoms off the up-stream.
		(define (make-down-stream)
			(if (not (wire-has-stream? up-wire))
				(error "Impossible condition: up-wire has no stream! -- cgw-xfer")
			)
			(set! input-stream (wire-take-stream up-wire))
			(if (stream-null? input-stream)
				(error "inut stream is unexpectedly empty - cgw-xfer")
			)
			; (list->stream (list 'a 'b 'c)) 
			(make-stream get-incoming '())
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
							; (stream-for-each do-stuff (wire-take-stream wire))
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
							(display "got stuff on up, neet to send down\n")
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

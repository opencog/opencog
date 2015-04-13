scm
;
; cgw-simple.scm
;
; Implements some of the simpleer devices, including sources,
; sinks and fanouts.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
; wire-null-device
;
; Generic null device. Ignores all messages.
;
(define (wire-null-device)

	; Trivial command dispatcher, accepts all commands and
	; does nothing at all.
	(define (me msg)
		'ok
	)

	; Return the command dispatcher.
	me
)

;; -------------------------------------------------------------------------
;
; wire-trivial-filter a-wire b-wire
;
; The trivial filter -- let everything on wire-a to pass to wire-b, 
; and vice-versa.

(define (wire-trivial-filter a-wire b-wire)
	(wire-filter a-wire b-wire (lambda (elt) #t))
)

; --------------------------------------------------------------------
; wire-drain wire
;
; Drain a stream from a wire.
; This is a "consumer" endpoint, it takes the stream off the wire
; and silently discards it.  (It consumes the stream). Do not use
; with infinite streams, or an infinite loop (hang) will result.
;
(define (wire-drain wire)
	(let ((myname ""))
		(define (me msg)
			(cond 
				((eq? msg wire-assert-msg)
					(stream-for-each (lambda (x) #f) (wire-take-stream wire)) 
				)
				((eq? msg wire-float-msg)
					; ignore this message
				)
				(else
					(default-dispatcher msg 'wire-drain myname)
				)
			)
		)

		; hook me up
		(wire-connect wire me)
	
		; return the command dispatcher.
		me
	)
)

; --------------------------------------------------------------------
; wire-probe probe-name wire
;
; Display a stream pulled from a wire.
; This is a "consumer" endpoint, it takes the stream off the wire
; and displays the contents of the stream.  (It consumes the stream).
;
; XXX Need to have a passive probe that can snoop on the wire.
; XXX this would probably have to be a special, snooping-wire.
;
(define (wire-probe probe-name wire)
	(define (prt-val value)
		(display "Probe: " )
		(display probe-name)
		(display " = ")
		(display value)
		(newline)
	)

	(define (me request)
		(cond 
			((eq? request wire-assert-msg)
				(stream-for-each prt-val (wire-take-stream wire)) 
			)
			((eq? request wire-float-msg)
				(prt-val "floating") 
			)
			(else
				(default-dispatcher request 'wire-probe probe-name)
			)
		)
	)

	; hook me up
	(wire-connect wire me)

	; return the command dispatcher.
	me
)

; --------------------------------------------------------------------
; clock-source wire
;
; Place a clock source on a wire
; This generates an infinite sequence of alternating #t, #f values
;
; Do *not* use this with the current opencog scheme interpreter.
; The problem is that this goes into an infinite loop. Worse,
; the output is buffered, and is invisible. Worse still, there's
; no way to ctrl-C this, since the shell server isn't properly
; threaded for i/o. Bummer. Should be fixed someday.
;
; XXX this device is more of an "example" device, it it not
; actually used anywhere, and probably shouldn't be used.
;
(define (clock-source wire)
	(define (toggle state) (cons state (not state)))

	(define (me request)
		(cond 
			((eq? request wire-float-msg) (lambda () #f)) ; ignore float message
			(else (error "Unknown request -- clock-source" request))
		)
		'ok
	)

	(wire-connect wire me)
	(wire-set-stream! wire (make-stream toggle #f) me)

	; return the command dispatcher.
	me
)

; --------------------------------------------------------------------
; wire-source-list wire lis
;
; Place a list onto a wire
; This creates a bus-master; the list will be clocked onto the bus.
(define (wire-source-list wire lis)

	(define (me request)
		(cond 
			((eq? request wire-assert-msg)
				(error "Cannot send data to a data source! -- wire-source-list"))
			((eq? request wire-float-msg) (lambda () #f)) ; ignore float message
			(else (error "Unknown message -- wire-source-list" request))
		)
		'ok
	)

	(wire-connect wire me)
	(wire-set-stream! wire (list->stream lis) me)

	; return the command dispatcher.
	me
)

; --------------------------------------------------------------------
.
exit

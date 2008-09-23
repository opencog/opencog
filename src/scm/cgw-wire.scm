scm
;
; cgw-wire.scm
;
; Implements a wire that can connect devices.
; See WIRING-README for overview.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; Implementation notes:
; By analogy to electrical engineering, can also think of wires as "buses";
; values are asserted onto the bus by one master, and all other parties
; on the bus must hold it "floating". Currently, the wire only grants 
; access to one bus-master; its might be possible to relax this.
; May need to create a specific grant/revoke protocal for the bus?
;
; Current implementation only allows *two* endpoints on the bus, not many.
; This is because the recever takes the stream from the sender. This seems
; like the most efficient way to proceed at the moment.
; The receiver is also called the "consumer", the transmitter is the "producer"
; Some of the words below refer to a bus, implying multiple endpoints, this
; no longer holds as the appropriate paradigm. -- The wire is purely 
; point-to-point. (Note, however, multiple devices are allowed to connect
; to the wire, if all that they want to do is to snoop for messages. However,
; only one device is allowed to take a stream ... since if two devices
; attempted to take the stream, there would be a race, and one would loose.)
;
; Unfortunately, ice-9 streams are significantly different than srfi-41
; streams, but, for expediancy, I'll be using ice-9 streams, for now.
; The biggest difference is that ice-9 doesn't have stream-cons, it has
; make-stream instead; so the semantics is different. Hopefully, porting 
; over won't be too hard.
;
(use-modules (ice-9 streams))
(define stream-null (make-stream (lambda (x) '()) '()))

; Simple example code:
;
; (define w (make-wire))
; (wire-probe "w-probe" w)
; (wire-source-list w (list 1 2 3 4 5))
;
; The above will create a wire w, attach a probe to the wire (the probe
; will print the values it sees on the wire) and then it will place a 
; sequence of numbers on the wire, which the probe will print out.
;
; Alternately, 
; (define w (make-wire "my whizbang wire" #t))
; will create a wire with a name, and with debugging turned on.

; Create a new wire
; Along the lines of "make-connector", SICP 3.3.5
(define (make-wire . rest)

	; Parse the optional arguments
	; Expect a wire-name as the first optional argument
	(define wire-name-arg
		(if (not (null? rest))
			(car rest)
			"default-wire-name"
		)
	)

	; Expect a debugging flag as the second optional argument
	(define debug-arg
		(if (not (null? rest))
			(if (not (null? (cdr rest)))
				(cadr rest)
				#f
			)
			#f
		)
	)

	(let (
		(debug-tracing debug-arg) ; set tracing from arg list
		(wire-dbg-name wire-name-arg) ; set wirename from arg
		(strm stream-null) ; only be streams are allowed
		(busmaster #f)    ; "informant" in SICP
		(endpoints '()))  ; "constraints" in SICP

		; Connect a new endpoint to the bus.
		; If theres' a master on the bus, then
		; let the new endpoint know about it.
		; XXX In the current design, the downstream device
		; always "takes" the stream, so in fact, the wire
		; can have only *two* enpoints, not many. The two
		; would be the master (source) and the slave (sink)
		(define (connect new-endpoint)
			(if (not (memq new-endpoint endpoints))
				(begin
					(set! endpoints (cons new-endpoint endpoints))

					(if debug-tracing
						(begin
							(display "Wire <")
							(display wire-dbg-name)
							(display "> connected to endpoint ")
							(display (wire-device-get-type new-endpoint))
							(display " <")
							(display (wire-device-get-name new-endpoint))
							(display ">\n")
						)
					)

					; If there is a master on this wire, 
					; then tell the new endpoint about it.
					(if (stream-null? strm)
						(float-msg new-endpoint)
						(deliver-msg new-endpoint)
					)
				)
			)
			'done
		)

		; Disconnect a wire
		(define (disconnect old-endpoint)
			(if (memq old-endpoint endpoints)
				(begin
					(if debug-tracing
						(begin
							(display "Wire <")
							(display wire-dbg-name)
							(display "> disconnected from endpoint ")
							(display (wire-device-get-type old-endpoint))
							(display " <")
							(display (wire-device-get-name old-endpoint))
							(display ">\n")
						)
					)
					(set! endpoints (delete! old-endpoint endpoints))
					(float-msg old-endpoint)
				)
			)
		)

		; Give the entire stream to the receving endpoint.
		; Clear the local copy, unset the busmaster
		(define (take-stream)
			(let ((local-strm strm))
				(set! busmaster #f)
				(set! strm stream-null)
			local-strm)

			; Note that when the stream is taken, the wire is left floating.
			; However, no float-msg is sent, since presumably the taker 
			; knows about it already ... 
		)

		; Let "master" assert a value onto the bus
		(define (set-value! newval master)

			(if debug-tracing
				(begin
					(display "Device ")
					(display (wire-device-get-type master))
					(display " <")
					(display (wire-device-get-name master))
					(display "> issued set-value on wire <")
					(display wire-dbg-name)
					(display ">\n")
				)
			)
			(if (not (stream-null? strm)) 
				(error "Stream already set on this wire" wire-dbg-name)
			)

			; The bus is floating, so grant
			(set! strm newval)
			(set! busmaster master)
			(for-each-except master deliver-msg endpoints)
		)

		(define (set-name! name)
			(set! wire-dbg-name name)
		)

		(define (me request)
			(cond 
				((eq? request 'value) (take-stream))
				((eq? request 'has-value?)
					(if busmaster #t #f)
				)
				((eq? request 'connect) connect)
				((eq? request 'disconnect) disconnect)
				((eq? request 'set-value!) set-value!)
				((eq? request 'debug-on!) (set! debug-tracing #t))
				((eq? request 'debug-off!) (set! debug-tracing #f))
				((eq? request 'set-name!) set-name!)
				(else (error "Unknown operation -- make-wire" wire-dbg-name request))
			)
		)

		; return the command dispatcher.
		me
	)
)

; --------------------------------------------------------------------
;
; return the value of the wire.
(define (wire-take-stream wire) (wire 'value))      ; SICP get-value
(define (wire-has-stream? wire) (wire 'has-value?)) ; SICP has-value?
(define (wire-set-stream! wire value endpoint)      ; SICP set-value!
	((wire 'set-value!) value endpoint)
)
(define (wire-connect wire endpoint)
	((wire 'connect) endpoint)
)
(define (wire-disconnect wire endpoint)
	((wire 'disconnect) endpoint)
)
(define (wire-enable-debug wire) (wire 'debug-on!))
(define (wire-disable-debug wire) (wire 'debug-off!))
(define (wire-set-name wire name)
	((wire 'set-name!) name)
)

; Basic messages
; "assert" means "there is a stream on this wire"
; "float"  means "there is no (longer any) stream on this wire"
(define wire-assert-msg 'I-want-to-xmit) ; I-have-a-value in SICP
(define wire-float-msg  'Ready-to-recv)  ; I-lost-my-value in SICP
(define wire-devname-msg  'get-device-name)
(define wire-setname-msg  'set-device-name)
(define wire-devtype-msg  'get-device-type)

(define (deliver-msg endpoint) (endpoint wire-assert-msg))
(define (float-msg endpoint) (endpoint wire-float-msg))

(define (default-dispatcher msg device devname)

	(define (set-name! name)
		(set! devname name)
	)

	; If the device dispatcher didn't handle the message,
	; then perhaps we can.
	(cond
		((eq? msg wire-devtype-msg) device)
		((eq? msg wire-devname-msg) devname)
		((eq? msg wire-setname-msg) set-name!)
		(else
			(error "Unkown device message " device devname msg)
		)
	)
)

; Basic device debugging support
(define (wire-device-get-type dev)
	(dev wire-devtype-msg)
)
(define (wire-device-get-name dev)
	(dev wire-devname-msg)
)
(define (wire-device-set-name! dev name)
	((dev wire-setname-msg) name)
)

; --------------------------------------------------------------------

.
exit

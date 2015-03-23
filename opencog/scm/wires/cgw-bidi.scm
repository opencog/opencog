scm
;
; cgw-bidi.scm
;
; Bidirectional device support.
; See WIRING-README for an overview.
;
; Copyright (c) 2008 Linas Vepstas <linasvepstas@gmail.com>
;
; --------------------------------------------------------------------
;
; wire-bidi a-wire b-wire uni-device
;
; Given a uni-directional device, build a bi-directional device.
; If 'uni-device' is a device that only takes input on the first wire,
; and only generates output on the second wire, then this device will
; implement the same function, but accept input on either wire, and 
; output on the other wire.
;
; This is implemented by listening for transmit messages. When
; a transmit message is overheard, then that wire is hooked up
; up to the uni-directional version, with the transmitter on the
; input side.
;
(define (wire-bidi a-wire b-wire uni-device)

	(let ((device (wire-null-device))
			(do-connect #t)
			(myname "")
		)

		(define (connect-me)
			; Two distinct checks of 'do-connect' are made, because the
			; act of connecting the a-wire could trigger a message that
			; hooks up the actual processor (and thus should leave 
			; wire-b not connected here)
			(if do-connect
				(begin
					(set! device me)
					(wire-connect a-wire me)
				)
			)
			(if do-connect
				(begin
					(set! device me)
					(wire-connect b-wire me)
				)
			)
		)

		(define (process-msg)
			;;
			;; State-change: Disconnect whateve the wires were previously connected to.
			(wire-disconnect a-wire device)
			(wire-disconnect b-wire device)
			(set! do-connect #f)
			(cond
				; Input on a-wire, output on b-wire. Disconnect ourself, connect
				; the uni-directional part in the right direction.
				((and (wire-has-stream? a-wire) (not (wire-has-stream? b-wire)))
					(set! device (uni-device a-wire b-wire))
				)

				; Input on b-wire, output on a-wire. Disconnect ourself, connect
				; the uni-directional part in the right direction.
				((and (not (wire-has-stream? a-wire)) (wire-has-stream? b-wire))
					(set! device (uni-device b-wire a-wire))
				)

				; Error condition
				((and (wire-has-stream? a-wire) (wire-has-stream? b-wire))
					(error "Both wires have streams! -- wire-bidi")
				)
			)
		)

		(define (process-disco-msg)
			(cond
				; Both wires floating. Disconnect the device, if previously attached,
				; and re-connect ourselves, so as to hear about anything new.
				((and 
						(not do-connect)
						(not (wire-has-stream? a-wire)) 
						(not (wire-has-stream? b-wire))
					)
					(wire-disconnect a-wire device)
					(wire-disconnect b-wire device)
					(set! do-connect #t)
					(connect-me)
				)
			)
		)

		(define (me msg)
			(cond
				((eq? msg wire-assert-msg)
					(process-msg)
				)
				((eq? msg wire-float-msg)
					(process-disco-msg)
				)
				(else
					(default-dispatcher msg 'wire-bidi myname)
 				)
			)
		)

		; Handy debugging prints
		; (set! myname "being-manually-debugged")
		; (wire-set-name a-wire "wire-bidi a-wire")
		; (wire-set-name b-wire "wire-bidi b-wire")
		; (wire-enable-debug a-wire)
		; (wire-enable-debug b-wire)

		(connect-me)

		; Return the dispatcher
		me
	)
)

; --------------------------------------------------------------------
.
exit

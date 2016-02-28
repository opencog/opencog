;
; Random under-documented unit-test scriptlet.
;
; Neither the vision subsystem, nor the chat subsystem needs to be
; started to run this test. Just start the behavior tree, then
; `rlwrap telnet localhost 17020` then
; `(load "unit-test.scm")`  and then
; `(run-face-test)`
;
; Run a unit test in a separate thread: add faces, chat, remove faces.
; These faces don't have corresponding 3D coords, the face-tracker will
; not know about them, so you can't use the facetracker while runing
; this test.
;
; Infinite loop, runs forever, unless you halt it by saying
; (set! keep-looping #f)

(use-modules (opencog eva-model)) ; defines `make-new-face`

(define do-print-msg #f)  ; say (set! do-print-msg #t) to print
(define keep-looping #t)  ; say (set! keep-looping #f) to halt unit test.

(define (run-face-test)
	(define face-id 0)

	(define (prt-msg x)
		(if do-print-msg (begin (display x) (force-output)))
	)

	(define (chat-for-a-while n)
		(do ((i 1 (1+ i))) ((> i n))
			(prt-msg "chat start\n")
			(StateLink chat-state chat-start)
			(sleep 1)
			(prt-msg "chat stop\n")
			(StateLink chat-state chat-stop)
			(sleep 2))
	)

	(define (come-and-go)
		(sleep 2)
		(set! face-id (+ 2 face-id))

		; Add one face
		(prt-msg "new face\n")
		(make-new-face (number->string face-id))
		(sleep 3)
		(chat-for-a-while 6)

		; Add a second face
		(prt-msg "second face\n")
		(make-new-face (number->string (+ 1 face-id)))
		(chat-for-a-while 6)
		(sleep 3)

		; Remove first face
		(prt-msg "first face exits\n")
		(remove-face (number->string face-id))
		(chat-for-a-while 6)

		; Remove second face
		(prt-msg "second face exits\n")
		(remove-face (number->string (+ 1 face-id)))

		; Let her get bored and look around and sleep.
		(sleep 90)

		; Loop forever.
		(if keep-looping (come-and-go))
	)

	; Run in own thread.
	(call-with-new-thread come-and-go)
)

; ----------------------------------------------------------
; Return count of total number of atoms in atomspace.
(define (get-total-atoms-in-atomspace)
	(define cnt 0)
	(define (ink a) (set! cnt (+ cnt 1)) #f)
	(define (cnt-type x) (cog-map-type ink x))
	(map cnt-type (cog-get-types))
	cnt)

; Print how many atoms don't have an incoming set.
(define (print-toplevel-counts)
	(define a-cnt 0)
	(define t-cnt 0)
	(define (ink a)
		(set! a-cnt (+ a-cnt 1))
		(if (eq? 0 (length (cog-incoming-set a)))
			(set! t-cnt (+ t-cnt 1)))
		#f)
	(define (cnt-type x)
		(set! a-cnt 0)
		(set! t-cnt 0)
		(cog-map-type ink x)
		(if (< 100 a-cnt)
			(display (format "type: ~A total ~A  top ~A\n" x a-cnt t-cnt)))
	)
	(for-each cnt-type (cog-get-types))
)

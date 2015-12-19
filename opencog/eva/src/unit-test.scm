;
; Random under-documented unit-test scriptlets.
;


; Run a unit test in a separate thread: add faces, chat, remove faces.
; These faces don't have corresponding 3D coords, the face-tracker will
; not know about them, so you can't use the facetracker while runing
; this test.
;
; Infinite loop, runs forever.
(define (run-face-test)
	(define face-id 0)

	(define (chat-for-a-while n)
		(do ((i 1 (1+ i))) ((> i n))
			(StateLink chat-state chat-start)
			(sleep 1)
			(StateLink chat-state chat-stop)
			(sleep 2))
	)

	(define (come-and-go)
		(sleep 2)
		(set! face-id (+ 2 face-id))

		; Add one face
		(make-new-face (number->string face-id))
		(sleep 3)
		(chat-for-a-while 6)

		; Add a second face
		(make-new-face (number->string (+ 1 face-id)))
		(chat-for-a-while 6)
		(sleep 3)

		; Remove first face
		(remove-face (number->string face-id))
		(chat-for-a-while 6)

		; Remove second face
		(remove-face (number->string (+ 1 face-id)))

		; Let her get bored and look around and sleep.
		(sleep 90)

		; Loop forever.
		(come-and-go)
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
	(map cnt-type (cog-get-types))
)

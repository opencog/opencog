;
; old-tree.scm
;
; Old (deprecated) top-level behavior tree main loop.
;
; This has been supplanted by the newer, OpenPsi-based behavior
; rules, and is temporarily maintained here for backwards
; compaitbility.
;
; HOWTO:
; ------
; Run the main loop:
;    (behavior-tree-run)
; Pause the main loop:
;    (behavior-tree-halt)
;
;; ------------------------------------------------------------------
;; Main loop. Uses tail recursion optimization to form the loop.
(DefineLink
	(DefinedPredicate "main loop")
	(SatisfactionLink
		(SequentialAnd
			(SequentialOr
				(DefinedPredicate "Skip Interaction?")

				(SequentialAnd
					(DefinedPredicate "Did Someone New Speak?")
					(DefinedPredicate "Request interaction with person who spoke"))

				(SequentialAnd
					(DefinedPredicate "Someone requests interaction?")
					(DefinedPredicate "Interaction requested action"))

				(SequentialAnd
					(DefinedPredicate "Did someone arrive?")
					(DefinedPredicate "New arrival sequence"))

				(SequentialAnd
					(DefinedPredicate "Did someone leave?")
					(DefinedPredicate "Someone left action"))

				; True, if there is anyone visible.
				(SequentialAnd
					(DefinedPredicate "Someone visible?")
					(DefinedPredicate "Interact with people"))

				(DefinedPredicate "Nothing is happening")
				(True))

			;; XXX FIXME chatbot is disengaged from everything else.
			;; The room can be empty, the head is bored or even asleep,
			;; but the chatbot is still smiling and yabbering.
			;; If interaction is turned-off need keep alive gestures
			(SequentialOr
				; If the TTS vocalization started (chatbot started talking) ...
				(SequentialAnd
					(DefinedPredicate "chatbot started talking?")
					(DefinedPredicate "Speech started"))

				; If the chatbot currently talking ...
				(SequentialAnd
					(DefinedPredicate "chatbot is talking?")
					(DefinedPredicate "Speech ongoing"))

				; If the chatbot stopped talking ...
				(SequentialAnd
					(DefinedPredicate "chatbot stopped talking?")
					(DefinedPredicate "Speech ended"))

				(SequentialAnd
					(DefinedPredicate "chatbot started listening?")
					(DefinedPredicate "Listening started"))

				; If the chatbot stopped talking ...
				(SequentialAnd
					(DefinedPredicate "chatbot is listening?")
					(DefinedPredicate "Listening ongoing"))

				; If the chatbot stopped talking ...
				(SequentialAnd
					(DefinedPredicate "chatbot stopped listening?")
					(DefinedPredicate "Listening ended"))

				(SequentialAnd
				    (DefinedPredicate "Skip Interaction?")
				    (DefinedPredicate "Keep alive"))

				(True)
			)

			; If ROS is dead, or the continue flag not set, then stop
			; running the behavior loop.
			(DefinedPredicate "Continue running loop?")
			(DefinedPredicate "ROS is running?")

			;; Call self -- tail-recurse.
			(DefinedPredicate "main loop")
		)))

; ----------------------------------------------------------------------
;; Main loop control
(define do-run-loop #t)

(define-public (behavior-tree-run)
"
 behavior-tree-run

 Run the Eva behavior tree main loop (in a new thread),
 Call (behavior-tree-halt) to exit the loop.
"
	(set! do-run-loop #t)
	(call-with-new-thread
		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop")))))

(define-public (behavior-tree-halt)
"
 behavior-tree-halt

 Tell the Eva behavior tree main loop thread to exit.
"
	(set! do-run-loop #f))


(define-public (behavior-tree-running?)
"
 behavior-tree-running?

 Return #t if the behavior tree is running, else return false.
"
	do-run-loop)

(define-public (behavior-tree-loop-count)
"
 behavior-tree-loop-count

 Return the loop-count of the behavior tree.
"
	loop-count)


(define loop-count 0)
(define-public (continue-running-loop)  ; public only because its in a GPN
	(set! loop-count (+ loop-count 1))

	; Print loop count to the screen.
	; (if (eq? 0 (modulo loop-count 30))
	;	(format #t "Main loop: ~a\n" loop-count))

	; Pause for one-tenth of a second... 101 millisecs
	(usleep 101000)
	(if do-run-loop (stv 1 1) (stv 0 1)))

; Return true if the behavior loop should keep running.
(DefineLink
	(DefinedPredicate "Continue running loop?")
	(Evaluation
		(GroundedPredicate "scm:continue-running-loop") (ListLink)))

; ----------------------------------------------------------------------

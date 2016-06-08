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

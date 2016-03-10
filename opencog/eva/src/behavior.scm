;
; behvaior.scm
;
; Configurable robot behavior tree, implemented in Atomese.
;
; Defines a set of behaviors that express Eva's personality. The
; currently-defined behaviors include acknowledging new people who enter
; the room, rotating attention between multiple occupants of the room,
; falling asleep when bored (i.e. the room is empty), and acting
; surprised when someone leaves unexpectedly.
;
; HOWTO:
; ------
; Run the main loop:
;    (behavior-tree-run)
; Pause the main loop:
;    (behavior-tree-halt)
;
; Unit testing:
; -------------
; The various predicates below can be manually unit tested by manually
; adding and removing new visible faces, and then manually invoking the
; various rules. See faces.scm for utilities.
;
; Basic examples:
;    Manually insert a face: (make-new-face id)
;    Remove a face: (remove-face id)
;    Etc.: (show-room-state) (show-eye-contact-state) (show-visible-faces)
;
; Manually unit test the new-arrival sequence.  You can do this without
; an attached camera; she won't track the face location, but should respond.
;    (make-new-face "42")
;    (cog-evaluate! (DefinedPredicateNode "New arrival sequence"))
;    (show-acked-faces)
;    (show-room-state)
;    (show-eye-contact-state)
;    (cog-evaluate! (DefinedPredicateNode "Interact with people"))
;
; Unit test the main interaction loop:
;    (run)  ; start the behavior tree running. Should print loop iteration.
;    (make-new-face "42")  ; Should start showing expressions, gestures.
;    (remove-face "42")  ; Should retur to neutral.
;    (halt) ; Pause the loop iteration; (run) will start it again.
;
; Unit test chatbot:
;    (State chat-state chat-start) ; to simulate having it talk.
;    (State chat-state chat-stop)  ; to signal that talking has stopped.
;
; --------------------------------------------------------
; Character engine notes
; The character engine is an older incarnation of this code, predating
; the owyl behavior trees.  The below are notes abouit some of the
; things it did, and where to look for equivalents here.
;
; 1) Room-state transtions
; -- no faces present for a while
;     (cog-evalute! (DefinedPredicateNode "Search for attention"))
; -- no-face just transitioned to one-face
;     (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
; -- no-face just transitioned to multiple-faces
; -- one-face just transitioned to no-face
; -- one-face just transitioned to multiple-faces
;      (DefinedPredicate "Respond to new arrival")
;      (DefinedPredicateNode "Interacting Sequence")
; -- one face present for a while
; -- multiple faces present for a while
; -- multiple-faces just transitioned to one-face
; -- multiple-faces just transitioned to no-face
;
; 2) Interactions depend on whether person is known or not.
;
; -- it's a familiar face
; -- it's a familiar face to whom the robot has already been introduced
; -- it's an unfamiliar face
;
; 3) Familiar-face interactions
;
; -- prior emotions when interacting with that face were positive
; -- prior emotions when interacting with that face were negative
; -- prior emotions when interacting with that face were neutral
;
; 4) Introduction sequence
;    If the robot is talking to someone with whom it has not previously
;    been introduced, it can enter into a short procedure aimed at
;    gathering the person's name...
;
; 5) Adhoc animation patterns
;
; -- changing a subject (maybe the AIML can send a signal indicating
;    when the subject is being changed?)
; -- asking a question of the user
; -- starting to talk after a period of being silent
; -- ending an interaction with one guy before shifting attention to
;    another guy
; -- starting an interaction with a new guy after shifting attention
;    from another guy
;
; --------------------------------------------------------
;
; This will be used to tag the name of the module making the request.
; Everything in ths module is the behavior tree, so that is what we
; call it.
(define bhv-source (Concept "Behavior Tree"))

; ------------------------------------------------------
; More complex interaction sequences.

;; Interact with the current face target.
;; XXX Needs to be replaced by OpenPsi emotional state modelling.
(DefineLink
	(DefinedPredicate "Interact with face")
	(SequentialAnd
		;; Look at the interaction face
		(True (DefinedSchema "look at person"))

		;; Show random expressions only if NOT talking
		(SequentialOr
			(Not (DefinedPredicate "chatbot is listening"))
			(SequentialAnd

				(SequentialOrLink
					(NotLink (DefinedPredicateNode "Time to change expression"))
					(DefinedPredicateNode "Show positive expression")
				)
				(SequentialOrLink
					(NotLink (DefinedPredicateNode "Time to make gesture"))
					(DefinedPredicateNode "Pick random positive gesture"))
		))
	))

; ------------------------------------------------------
;; Was Empty Sequence - if there were no people in the room, then
;; look at the new arrival.
;;
;; (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
(DefineLink
	(DefinedPredicate "Was Empty Sequence")
	(SequentialAnd
		(DefinedPredicate "was room empty?")
		; Record a new emotional state (for self-awareness)
		; XXX FIXME this should be a prt of "Show random expression"
		; below ...
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (Concept "new-arrival")))

		(DefinedPredicate "interact with new person")
		(TrueLink (DefinedSchema "look at person"))
		(PutLink (DefinedPredicate "Show random expression")
			(ConceptNode "new-arrival"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Look at newly arrived person")))
	))

;; If interaction is requested, then interact with that specific person.
;; Make sure we look at that person ...
(DefineLink
	(DefinedPredicate "Interaction requested")
	(SequentialAnd
		(DefinedPredicate "Someone requests interaction?")
		(True (DefinedPredicate "If sleeping then wake"))
		(True (DefinedPredicate "If bored then alert"))
		(DefinedPredicate "interact with requested person")
		(True (DefinedSchema "look at person"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Looking at requested face")))
	))

;; Sequence - Currently interacting with someone when a new person
;  becomes visible.
; (cog-evaluate! (DefinedPredicateNode "Interacting Sequence"))
(DefineLink
	(DefinedPredicateNode "Interacting Sequence")
	(SequentialAndLink
		(DefinedPredicateNode "is interacting with someone?")
		(DefinedPredicateNode "dice-roll: glance new face")
		(TrueLink (DefinedSchemaNode "glance at new person"))
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Glance at new person")))
	))

;; Respond to a new face becoming visible.
;
;; XXX TODO -- need to also do line 590, if interacting for a while
;; this alters probability of glance...
(DefineLink
	(DefinedPredicate "Respond to new arrival")
	(SequentialOr
		(DefinedPredicate "Was Empty Sequence")
		(DefinedPredicate "Interacting Sequence")
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Ignoring new person")))
		(True)
	))

;; Return false if not sleeping.
;; Return true if we were sleeping, adn we woke up.
(DefineLink
	(DefinedPredicate "If sleeping then wake")
	(SequentialAnd
		(DefinedPredicate "Is sleeping?")
		(DefinedPredicate "Wake up")))

;; If soma state was bored, change state to alert.
;; If we don't do this, then being bored while also talking
;; can make us narcoleptic.
;;
;; Return false if not bored.
;; Return true if we were bored, adn we woke up.
(DefineLink
	(DefinedPredicate "If bored then alert")
	(SequentialAnd
		(DefinedPredicate "Is bored?")
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-awake))))

;; Check to see if a new face has become visible.
(DefineLink
	(DefinedPredicate "New arrival sequence")
	(SequentialAnd
		(DefinedPredicate "Did someone arrive?")
		(True (DefinedPredicate "If sleeping then wake"))
		(True (DefinedPredicate "If bored then alert"))
		(DefinedPredicate "Respond to new arrival")
		(DefinedPredicate "Update status")
	))

;; Check to see if someone left.
(DefineLink
	(DefinedPredicateNode "Someone left")
	(SequentialAndLink
		(DefinedPredicateNode "Did someone leave?")
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Someone left")))
		(SequentialOrLink
			; Were we interacting with the person who left? If so,
			; look frustrated, return head position to neutral.
			(SequentialAndLink
				(EqualLink
					(DefinedSchemaNode "New departures")
					(GetLink (StateLink eye-contact-state (VariableNode "$x"))))
				(DefinedPredicateNode "Show frustrated expression")
				(DefinedPredicateNode "return to neutral")
			)
			;; Were we interacting with someone else?  If so, then
			;; maybe glance at the location of the person who left.
			(SequentialAndLink
				(DefinedPredicateNode "is interacting with someone?")
				(SequentialOrLink
					(NotLink (DefinedPredicateNode "dice-roll: glance lost face"))
					(FalseLink (DefinedSchemaNode "glance at lost face"))
					(EvaluationLink (GroundedPredicateNode "scm: print-msg")
						(ListLink (Node "--- Glance at lost face"))))
				(TrueLink)
			)
			(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				(ListLink (Node "--- Ignoring lost face")))
			(TrueLink)
		)
		;; Clear the lost face target
		(DefinedPredicateNode "Clear lost face")
		(DefinedPredicateNode "Update room state")
	))

;; Collection of things to do while interacting with people.
;; Evaluates to true if there is an ongoing interaction with
;; someone.
(DefineLink
	(DefinedPredicate "Interact with people")
	(SequentialAnd
		; True, if there is anyone visible.
		(DefinedPredicate "Someone visible")
		; This sequential-or is true if we're not interacting with anyone,
		; or if there are several people and its time to change up.
		(SequentialOr
			; Start a new interaction, but only if not currently interacting.
			(SequentialAnd
				(SequentialOr
					(Not (DefinedPredicate "is interacting with someone?"))
					(SequentialAnd
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "Time to change interaction")))
				; Select a new face target, interact with it.
				(DefinedPredicate "Change interaction")
				(DefinedPredicate "Interact with face"))

			; ##### Glance At Other Faces & Continue With The Last Interaction
			(SequentialAnd
				; Gets called 10x/second; don't print.
				;(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				;	(ListLink (Node "--- Continue interaction")))
				(SequentialOr
					(SequentialAnd
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "dice-roll: group interaction")
						(DefinedPredicate "glance at random face"))
					(True))
				(DefinedPredicateNode "Interact with face")
				(SequentialOr
					(SequentialAnd
						(DefinedPredicateNode "dice-roll: face study")
; XXX incomplete!  need the face study saccade stuff...
; I am confused ... has this been superceeded by the ROS-saccades,
; or is this still means to be used?
						(False)
					)
					(True))
			))
	))

; ------------------------------------------------------
; Empty-room behaviors. We either search for attention, or we sleep,
; or we wake up.

(DefineLink
	(DefinedPredicateNode "Search for attention")
	(SequentialAndLink
		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "bored")))

		; Pick a bored expression, gesture
		(SequentialOr
			(Not (DefinedPredicate "Time to change expression"))
			(PutLink (DefinedPredicateNode "Show random expression")
				(ConceptNode "bored")))
		(SequentialOr
			(Not (DefinedPredicate "Time to make gesture"))
			(PutLink (DefinedPredicateNode "Show random gesture")
				(ConceptNode "bored")))

		;; Search for attention -- change gaze every so often.
		;; Coordinate system: x forward; y side-to-side, z up.
		;; XXX question: This is turning the whole head; perhaps we
		;; should be moving eyes only?
		(SequentialOr
			(Not (DefinedPredicate "Time to change gaze"))
			(SequentialAnd
				(Evaluation (DefinedPredicate "Look at point")
					(ListLink ;; three numbers: x,y,z
						(Number 1)
						(RandomNumber
							(DefinedSchema "gaze right max")
							(DefinedSchema "gaze left max"))
						(Number 0)))
				(TrueLink (DefinedSchemaNode "set attn-search timestamp"))
			))
	))

; Call once, to fall asleep.
(DefineLink
	(DefinedPredicate "Go to sleep")
	(SequentialAnd
		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "sleepy")))

		; Proceed with the sleep animation only if the state
		; change was approved.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-sleeping))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Go to sleep.")
				(Minus (TimeLink) (DefinedSchema "get bored timestamp"))))
		(True (DefinedSchema "set sleep timestamp"))

		; First, show some yawns ...
		(Put (DefinedPredicate "Show random expression")
			(Concept "sleepy"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "sleepy"))

		; Finally, play the go-to-sleep animation.
		(Evaluation (GroundedPredicate "py:do_go_sleep") (ListLink))
	))

; Continue To Sleep
(DefineLink
	(DefinedPredicateNode "Continue sleeping")
	(SequentialAndLink
		(TrueLink (DefinedSchemaNode "set bored timestamp"))
		;(EvaluationLink (GroundedPredicateNode "scm: print-msg")
		;	(ListLink (Node "--- Continue sleeping.")))
	))

; Wake-up sequence
(DefineLink
	(DefinedPredicate "Wake up")
	(SequentialAnd
		; Request change soma state to being awake. Proceed only if
		; the request is accepted.
		(Evaluation (DefinedPredicate "Request Set Soma State")
			(ListLink bhv-source soma-awake))

		; Proceed only if we are allowed to.
		(Put (DefinedPredicate "Request Set Emotion State")
			(ListLink bhv-source (ConceptNode "wake-up")))

		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Wake up!")
				(Minus (TimeLink) (DefinedSchema "get sleep timestamp"))))
		(TrueLink (DefinedSchema "set bored timestamp"))

		; Run the wake animation.
		(Evaluation (GroundedPredicate "py:do_wake_up") (ListLink))

		; Also show the wake-up expression (head shake, etc.)
		(Put (DefinedPredicate "Show random expression")
			(Concept "wake-up"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "wake-up"))
	))

;; Collection of things to do if nothing is happening (no faces
;; are visibile)
;; Go to sleep after a while, and wake up every now and then.
(DefineLink
	(DefinedPredicate "Nothing is happening")
	(SequentialAnd

		; If we are not bored already, and we are not sleeping,
		; then we are bored now...
		(SequentialOr
			(DefinedPredicate "Is bored?")
			(DefinedPredicate "Is sleeping?")

			(SequentialAnd
				(Evaluation (DefinedPredicate "Request Set Soma State")
					(ListLink bhv-source soma-bored))

				(True (DefinedSchema "set bored timestamp"))

				; ... print output.
				(Evaluation (GroundedPredicate "scm: print-msg")
					(ListLink (Node "--- Bored! nothing is happening!")))
			))

		(SequentialOr
			; ##### Is Not Sleeping #####
			(SequentialAnd
				; Proceed only if not sleeping ...
				(Not (DefinedPredicate "Is sleeping?"))

				(SequentialOr
					; ##### Go To Sleep #####
					(SequentialAnd
						(DefinedPredicate "Bored too long")
						(DefinedPredicate "Go to sleep"))

					; ##### Search For Attention #####
					; If we didn't fall asleep above, then search for attention.
					(DefinedPredicate "Search for attention")
				))
			; ##### Is Sleeping #####
			(SequentialOr
				; ##### Wake Up #####
				(SequentialAnd
					; Did we sleep for long enough?
					(DefinedPredicate "Time to wake up")
					(DefinedPredicate "Wake up")
				)
				; ##### Continue To Sleep #####
				(DefinedPredicate "Continue sleeping")
			)
		)
))

;; ------------------------------------------------------------------
;; Chat-related behaviors.
;;
;; These can be unit-tested by saying
;;    rostopic pub --once chat_events std_msgs/String speechstart
;;    rostopic pub --once chat_events std_msgs/String speechend

; Things to do, if TTS vocalization just started.
(DefineLink
	; owyl "chatbot_speech_start()" method
	(DefinedPredicate "Speech started?")
	(SequentialAnd
		; If the TTS vocalization started (chatbot started talking) ...
		(DefinedPredicate "chatbot started talking")
		; ... then switch to face-study saccade ...
		(Evaluation (GroundedPredicate "py:conversational_saccade")
				(ListLink))
		; ... and show one random gesture from "conversing" set.
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "conversing"))
		; ... and also, sometimes, the "chatbot_positive_nod"
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "chat-positive-nod"))
		; ... and switch state to "talking"
		(True (Put (State chat-state (Variable "$x")) chat-talk))

		; ... print output.
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Start talking")))
))

;; Things to do, if the chatbot is currently talking.
(DefineLink
	(DefinedPredicate "Speech ongoing?")
	(SequentialAnd
		; If the chatbot currently talking ...
		(DefinedPredicate "chatbot is talking")
		; ... then handle the various affect states.
		(SequentialOr
			(SequentialAnd
				; If chatbot is happy ...
				(DefinedPredicate "chatbot is happy")
				; ... show one of the neutral-speech expressions
				(SequentialOr
					(Not (DefinedPredicate "Time to change expression"))
					(Put (DefinedPredicateNode "Show random expression")
						(ConceptNode "neutral-speech")))

				; ... nod slowly ...
				(SequentialOr
					(Not (DefinedPredicate "Time to make gesture"))
					(SequentialAnd
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-positive-nod"))

						; ... raise eyebrows ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-pos-think"))

						; ... switch to chat fast blink rate...
						(Evaluation (GroundedPredicate "py:blink_rate")
							(ListLink
								(DefinedSchema "blink chat fast mean")
								(DefinedSchema "blink chat fast var")))
				))
			)
			(SequentialAnd
				; If chatbot is not happy ...
				(DefinedPredicate "chatbot is negative")
				; ... show one of the frustrated expressions
				(SequentialOr
					(Not (DefinedPredicate "Time to change expression"))
					(Put (DefinedPredicateNode "Show random expression")
						(ConceptNode "frustrated")))
				(SequentialOr
					(Not (DefinedPredicate "Time to make gesture"))
					(SequentialAnd
						; ... shake head quickly ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-negative-shake"))
						; ... furrow brows ...
						(Put (DefinedPredicate "Show random gesture")
							(ConceptNode "chat-neg-think"))
						; ... switch to chat slow blink rate...
						(Evaluation (GroundedPredicate "py:blink_rate")
							(ListLink
								(DefinedSchema "blink chat slow mean")
								(DefinedSchema "blink chat slow var")))
				))
			))))

; Things to do, if the chattbot stopped talking.
(DefineLink
	(DefinedPredicate "Speech ended?")
	(SequentialAnd
		; If the chatbot stopped talking ...
		(DefinedPredicate "chatbot stopped talking")

		; ... then switch back to exploration saccade ...
		(Evaluation (GroundedPredicate "py:explore_saccade")
			(ListLink))

		; ... switch to normal blink rate...
		(Evaluation (GroundedPredicate "py:blink_rate")
			(ListLink
				(DefinedSchema "blink normal mean")
				(DefinedSchema "blink normal var")))

		; ... and switch state to "listening"
		(True (Put (State chat-state (Variable "$x")) chat-listen))

		; ... and print some tracing output
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Finished talking")))
	))

; Things to do, if the chatbot is listening.
(DefineLink
	(DefinedPredicate "Speech listening?")
	(SequentialAnd
		; If the chatbot stopped talking ...
		(DefinedPredicate "chatbot is listening")

		; No-op.  What should we do here?
		(TrueLink)
	))

;; ------------------------------------------------------------------
;; Main loop. Uses tail recursion optimization to form the loop.
(DefineLink
	(DefinedPredicate "main loop")
	(SatisfactionLink
		(SequentialAnd
			(SequentialOr
				(DefinedPredicate "Interaction requested")
				(DefinedPredicate "New arrival sequence")
				(DefinedPredicate "Someone left")
				(DefinedPredicate "Interact with people")
				(DefinedPredicate "Nothing is happening")
				(True))

			;; XXX FIXME chatbot is disengaged from everything else.
			;; The room can be empty, the head is bored or even asleep,
			;; but the chatbot is still smiling and yabbering.
			(SequentialOr
				(DefinedPredicate "Speech started?")
				(DefinedPredicate "Speech ongoing?")
				(DefinedPredicate "Speech ended?")
				; (DefinedPredicate "Speech listening?") ; no-op
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

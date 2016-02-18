;
; btree.scm
;
; Behavior tree in the atomspace.
; Under construction.
;
; Run the main loop:
;    (run)
; Pause the main loop:
;    (halt)
;
; Unit testing:
; The various predicates below can be manually unit tested by manually
; adding and removing new visible faces, and then manually invoking the
; various rules. See faces.scm for utilities:
;
; Manually insert a face: (make-new-face id)
; Remove a face: (remove-face id)
; Etc.: (show-room-state) (show-interaction-state) (show-visible-faces)
;
; Unit test the new-arrival sequence:
; (make-new-face "42")
; (cog-evaluate! (DefinedPredicateNode "New arrival sequence"))
; (show-acked-faces)
; (show-room-state)
; (show-interaction-state)
; (cog-evaluate! (DefinedPredicateNode "Interact with people"))
;
; Unit test the main interaction loop:
; (run)  ; start the behavior tree running. Should print loop iteration.
; (make-new-face "42")  ; Should start showing expressions, gestures.
; (remove-face "42")  ; Should retur to neutral.
; (halt) ; Pause the loop iteration; (run) will start it again.
;
; Unit test chatbot:
; (State chat-state chat-start) ; to simulate having it talk.
; (State chat-state chat-stop)  ; to signal that talking has stopped.

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

(system "echo \"py\\n\" | cat - atomic.py |netcat localhost 17020")
; (system "echo \"py\\n\" | cat - atomic-dbg.py |netcat localhost 17020")

(load-from-path "utilities.scm")

; (display %load-path)
(add-to-load-path "../src")
(load-from-path "faces.scm")
(load-from-path "cfg-tools.scm")
(load-from-path "behavior-cfg.scm")
(load-from-path "self-model.scm")

; (use-modules (opencog logger))
; (cog-logger-set-stdout #t)

; --------------------------------------------------------
; Some debug prints.

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "~a\n" atom) (stv 1 1))

; Print message, and print the current interaction face-id
(define (print-msg-face node)
	(display (cog-name node))
	(display " with face id: ")
	(display (cog-name (car (cog-outgoing-set (cog-execute!
			(DefinedSchemaNode "Current interaction target"))))))
	(newline)
	(stv 1 1))

; Print message, then print elapsed time
(define (print-msg-time node time)
	(display (cog-name node))
	(display " Elapsed: ")
	(display (cog-name time))
	(display " seconds\n")
	(stv 1 1))

; --------------------------------------------------------

; Start interacting with a new face picked randomly from the crowd.
(DefineLink
	(DefinedPredicateNode "Start new interaction")
	(SequentialAndLink
		; First, pick a face at random...
		(TrueLink (PutLink
			(StateLink interaction-state (VariableNode "$face-id"))
			(DefinedSchemaNode "Select random face")))
		; Record a timestamp
		(TrueLink (DefinedSchemaNode "set interaction timestamp"))
		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Start new interaction")))
	))

;; ------

;; line 809 + line 483 -- glance_at(id="current_glance_target")
(DefineLink
	(DefinedPredicateNode "glance at random face")
	(SequentialAndLink
		(DefinedPredicateNode "Select random glance target")
		(TrueLink (PutLink
			(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
				(ListLink (VariableNode "$face")))
			(GetLink (StateLink glance-state (VariableNode "$face-id")))
		))))

;; line 818 -- glance_at_new_face()
(DefineLink
	(DefinedSchema "glance at new person")
	(Put
		(Evaluation (GroundedPredicate "py:glance_at_face")
			(ListLink (Variable "$face")))
		; If more than one new arrival, pick one randomly.
		(RandomChoice (DefinedSchema "New arrivals"))
	))

;; line 827 -- glance_at_lost_face()
(DefineLink
	(DefinedSchemaNode "glance at lost face")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New departures")))


; ------------------------------------------------------
; More complex interaction sequences.


;; Interact with the curent face target.
;; line 762, interact_with_face_target()
;; XXX Needs to be replaced by OpenPsi emotional state modelling.
(DefineLink
	(DefinedPredicate "Interact with face")
	(SequentialAnd
		;; Look at the interaction face - line 765
		(True (DefinedSchema "look at person"))

		;; Show random expressions only if NOT talking
		; aka "do_pub_emotions=False" in the new owyl tree.
		(SequentialOr
			(Not (DefinedPredicate "chatbot is listening"))
			(SequentialAnd

				;; line 768
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
;; Sequence - if there were no people in the room, then look at the
;; new arrival.
;; line 391 -- owyl.sequence
;; (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
(DefineLink
	(DefinedPredicate "Was Empty Sequence")
	(SequentialAnd
		;; line 392
		(DefinedPredicateNode "was room empty?")
		(TrueLink (DefinedSchemaNode "interact with new person"))
		(TrueLink (DefinedSchemaNode "look at person"))
		(TrueLink (DefinedSchemaNode "set interaction timestamp"))
		(PutLink (DefinedPredicateNode "Show random expression")
			(ConceptNode "new-arrival"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Look at newly arrived person")))
	))

;; If interaction is requested, then interact wwith that person.
;; Make sure we look at that person and restart the interaction timer.
(DefineLink
	(DefinedPredicate "Interaction requested")
	(SequentialAnd
		(DefinedPredicate "Someone requests interaction?")
		(DefinedPredicate "If sleeping then wake")
		(DefinedPredicate "If bored then alert")
		(True (DefinedSchema "interact with requested person"))
		(True (DefinedSchema "clear requested face"))
		(True (DefinedSchema "look at person"))
		(True (DefinedSchema "set interaction timestamp"))
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Looking at requested face")))
	))

;; line 399 -- Sequence - Currently interacting with someone
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
;; line 389 -- Selector
;
;; XXX TODO -- need to also do line 590, if interacting for a while
;; this alters probability of glance...
(DefineLink
	(DefinedPredicate "Respond to new arrival")
	(SequentialOr
		(DefinedPredicate "Was Empty Sequence")
		(DefinedPredicate "Interacting Sequence")
		(Evaluation (GroundedPredicate "scm: print-msg")
			(ListLink (Node "--- Ignoring new person"))) ; line 406
		(True)
	))

;; ##### If Interruption && Sleeping -> Wake Up #####
;; line 545
(DefineLink
	(DefinedPredicate "If sleeping then wake")
	(SequentialOr
		(SequentialAnd
			(Equal (SetLink soma-sleeping)
				(Get (State soma-state (Variable "$x"))))
			(DefinedPredicateNode "Wake up"))
		(True)))

;; If soma state was bored, change state to alert.
;; If we don't do this, then being bored while also talking
;; can make us narcoleptic.
(DefineLink
	(DefinedPredicate "If bored then alert")
	(SequentialOr
		(NotLink (Equal (SetLink soma-bored)
			(Get (State soma-state (Variable "$x")))))
		(True (Put (State soma-state (Variable "$x")) soma-awake))))

;; Check to see if a new face has become visible.
;; line 386 -- someone_arrived()
(DefineLink
	(DefinedPredicate "New arrival sequence")
	(SequentialAnd
		(DefinedPredicate "Did someone arrive?")
		(DefinedPredicate "If sleeping then wake")
		(DefinedPredicate "If bored then alert")
		(DefinedPredicate "Respond to new arrival")
		(DefinedPredicate "Update status")
	))

;; Check to see if someone left.
;; line 422 -- someone_left()
(DefineLink
	(DefinedPredicateNode "Someone left")
	(SequentialAndLink
		(DefinedPredicateNode "Did someone leave?")
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Someone left")))
		(SequentialOrLink
			; Were we interacting with the person who left? If so,
			; look frustrated, return to neutral. Oh, and clear the
			; interaction target, too.
			(SequentialAndLink
				(EqualLink
					(DefinedSchemaNode "New departures")
					(GetLink (StateLink interaction-state (VariableNode "$x"))))
				(DefinedPredicateNode "Show frustrated expression")
				(DefinedPredicateNode "return to neutral")
				(TrueLink (PutLink
					(StateLink interaction-state (VariableNode "$face-id"))
					no-interaction))
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

;; Collection of things to while interacting with people
;; Evalutes to true if there is an ongoing interaction with
;; someone.
;; line 457 -- interact_with_people()
(DefineLink
	(DefinedPredicate "Interact with people")
	(SequentialAnd ; line 458
		; True, if there is anyone visible.
		(DefinedPredicate "Someone visible") ; line 459
		; This sequential-or is true if we're not interacting with anyone,
		; or if there are several people and its time to change up.
		(SequentialOr ; line 460
			; ##### Start A New Interaction #####
			(SequentialAnd ; line 462
				(SequentialOr ; line 463
					(Not (DefinedPredicate "is interacting with someone?"))
					(SequentialAnd ; line 465
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "Time to change interaction")))
				; Select a new face target
				(DefinedPredicate "Start new interaction")
				(DefinedPredicate "Interact with face"))

			; ##### Glance At Other Faces & Continue With The Last Interaction
			(SequentialAnd ; line 476
				; Gets called 10x/second; don't print.
				;(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				;	(ListLink (Node "--- Continue interaction")))
				(SequentialOr  ; line 478
					(SequentialAnd ; line 479
						(DefinedPredicate "More than one face visible")
						(DefinedPredicate "dice-roll: group interaction")
						(DefinedPredicate "glance at random face"))
					(True)) ; line 485
				(DefinedPredicateNode "Interact with face")
				(SequentialOr  ; line 488
					(SequentialAnd ; line 489
						(DefinedPredicateNode "dice-roll: face study")
; XXX incomplete!  need the face study saccade stuff...
; I am confused ... has this been superceeded by the ROS-saccades,
; or is this still means to be used?
						(False)
					)
					(True))  ; line 493
			))
	))

; ------------------------------------------------------
; Empty-room behaviors. We either search for attention, or we sleep,
; or we wake up.

; line 898 -- search_for_attention.
(DefineLink
	(DefinedPredicateNode "Search for attention")
	(SequentialAndLink
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
				(Evaluation (GroundedPredicate "py:look_at_point")
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
; line 941 -- go_to_sleep
(DefineLink
	(DefinedPredicate "Go to sleep")
	(SequentialAnd
		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Go to sleep.")
				(Minus (TimeLink) (DefinedSchema "get bored timestamp"))))
		(True (DefinedSchema "set sleep timestamp"))

		; First, show some yawns ...
		(Put (DefinedPredicate "Show random expression")
			(Concept "sleep"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "sleep"))

		; Finally, play the go-to-sleep animation.
		(Evaluation (GroundedPredicate "py:do_go_sleep") (ListLink))
		(True (Put (State soma-state (VariableNode "$x")) soma-sleeping))
	))

; line 537 -- Continue To Sleep
(DefineLink
	(DefinedPredicateNode "Continue sleeping")
	(SequentialAndLink
		(TrueLink (DefinedSchemaNode "set bored timestamp"))
		;(EvaluationLink (GroundedPredicateNode "scm: print-msg")
		;	(ListLink (Node "--- Continue sleeping.")))
	))

; Wake-up sequence
; line 957 -- wake_up()
(DefineLink
	(DefinedPredicate "Wake up")
	(SequentialAnd
		(Evaluation (GroundedPredicate "scm: print-msg-time")
			(ListLink (Node "--- Wake up!")
				(Minus (TimeLink) (DefinedSchema "get sleep timestamp"))))
		(TrueLink (DefinedSchema "set bored timestamp"))

		; Change soma state to being awake.
		(True (Put (State soma-state (Variable "$x")) soma-awake))

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
;; line 507 -- nothing_is_happening()
(DefineLink
	(DefinedPredicate "Nothing is happening")
	(SequentialAnd  ; line 508

		; If we are not bored already, and we are not sleeping,
		; then we are bored now...
		(SequentialOr
			(Equal (SetLink soma-bored)
				(Get (State soma-state (Variable "$x"))))

			(Equal (SetLink soma-sleeping)
				(Get (State soma-state (Variable "$x"))))

			(SequentialAnd
				(True (Put (State soma-state (Variable "$x")) soma-bored))

				(True (DefinedSchema "set bored timestamp"))

				; ... print output.
				(Evaluation (GroundedPredicate "scm: print-msg")
					(ListLink (Node "--- Bored! nothing is happening!")))
			))

		(SequentialOr  ; line 509
			; ##### Is Not Sleeping #####
			(SequentialAnd ; line 511
				; Proceed only if not sleeping ...
				(Not (Equal (SetLink soma-sleeping)
					(Get (State soma-state (Variable "$x")))))

				(SequentialOr  ; line 513
					; ##### Go To Sleep #####
					(SequentialAnd  ; line 515
						(DefinedPredicate "Bored too long")
						(DefinedPredicate "Go to sleep"))

					; ##### Search For Attention #####
					; If we didn't fall asleep above, then search for attention.
					(DefinedPredicate "Search for attention")
				))
			; ##### Is Sleeping #####
			(SequentialOr  ; line 528
				; ##### Wake Up #####
				(SequentialAnd  ; line 530
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

; Things to do, if the chatbot started talking.
(DefineLink
	; owyl "chatbot_speech_start()" method
	(DefinedPredicate "Speech started?")
	(SequentialAnd
		; If the chatbot started talking ...
		(DefinedPredicate "chatbot started talking")
		; ... then switch to face-study saccade ...
		(Evaluation (GroundedPredicate "py:conversational_saccade")
				(ListLink))
		; ... and show one random gesture from "listening" set.
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "listening"))
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

; Things to do, if the chattbot is listening.
(DefineLink
	(DefinedPredicate "Speech listening?")
	(SequentialAnd
		; If the chatbot stopped talking ...
		(DefinedPredicate "chatbot is listening")

		; No-op. The current owyl tree does nothing here.
		(TrueLink)
	))

;; ------------------------------------------------------------------
;; Main loop diagnostics
;; line 988 - idle_spin()
(define loop-count 0)
(define do-run-loop #t)
(define (idle-loop)
	(set! loop-count (+ loop-count 1))

	(if (eq? 0 (modulo loop-count 30))
		(format #t "Main loop: ~a\n" loop-count))

	; Pause for one-tenth of a second... 101 millisecs
	(usleep 101000)
	(if do-run-loop (stv 1 1) (stv 0 1)))

;; Main loop. Uses tail recursion optimization to form the loop.
;; line 556 -- build_tree()
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
			;; The room can be empty, the head is bored or even sleep,
			;; but the chatbot is still smiling and yabbering.
			(SequentialOr
				(DefinedPredicate "Speech started?")
				(DefinedPredicate "Speech ongoing?")
				(DefinedPredicate "Speech ended?")
				; (DefinedPredicate "Speech listening?") ; no-op
				(True)
			)
			(Evaluation
				(GroundedPredicate "scm:idle-loop") (ListLink))
			(Evaluation
				(GroundedPredicate "py:ros_is_running") (ListLink))

			;; Call self -- tail-recurse.
			(DefinedPredicate "main loop")
		)))

;; Run the loop (in a new thread)
;; Call (run) to run the loop, (halt) to pause the loop.
;; line 297 -- self.tree.next()
(define (run)
	(set! do-run-loop #t)
	(call-with-new-thread
		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop")))))
(define (halt) (set! do-run-loop #f))

; Sigh. Perform manual garbage collection. This really should be
; automated. XXX TODO. (Can we get ECAN to do this for us?)
(define (run-atomspace-gc)
	(define (free-stuff)
		(sleep 1)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'SetLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ListLink)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'NumberNode)
		(cog-map-type (lambda (a) (cog-delete a) #f) 'ConceptNode)
		(free-stuff)
	)
	(call-with-new-thread free-stuff)
)

; Run the gc too...
(run-atomspace-gc)

;
; Silence the output.
(TrueLink)

;; Actually set it running
(all-threads)
(run)

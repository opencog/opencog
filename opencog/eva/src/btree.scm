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

(use-modules (opencog logger))
; (cog-logger-set-stdout #t)


; ------------------------------------------------------
; State variables

; Soma: awake, agitated, excited, tired, manic, depressed
(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))
(define soma-awake (ConceptNode "Awake"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; Currently, interaction-state will be linked to the face-id of
;; person with whom interaction is taking place. (current_face_target in owyl)
(define interaction-state (AnchorNode "Interaction State"))
(define no-interaction (ConceptNode "none"))

(StateLink interaction-state no-interaction)
(StateLink (SchemaNode "start-interaction-timestamp") (NumberNode 0))
(StateLink (SchemaNode "start-expression-timestamp") (NumberNode 0))
(StateLink (SchemaNode "gesture-timestamp") (NumberNode 0))
(StateLink (SchemaNode "start-sleep-timestamp") (NumberNode 0))
(StateLink (SchemaNode "start-boredom-timestamp") (NumberNode 0))

; The face to glance at.
(define glance-state (AnchorNode "Glance State"))
(StateLink glance-state no-interaction)

; Chat state. Is the robot talking, or not, right now?
; NB the python code uses these defines!
(define chat-state (AnchorNode "Chat State"))
(define chat-listen (ConceptNode "Listening"))
(define chat-talk (ConceptNode "Talking"))
(define chat-start (ConceptNode "Start Talking"))
(define chat-stop (ConceptNode "Stop Talking"))
(StateLink chat-state chat-stop)

(DefineLink
	(DefinedPredicate "chatbot started talking")
	(Equal (Set chat-start)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is talking")
	(Equal (Set chat-talk)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot stopped talking")
	(Equal (Set chat-stop)
		(Get (State chat-state (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is listening")
	(Equal (Set chat-listen)
		(Get (State chat-state (Variable "$x")))))

; Chat affect. Is the robot happy about what its saying?
; Right now, there are only two affects: happy and not happy.
; NB the python code uses these defines!
(define chat-affect (AnchorNode "Chat Affect"))
(define chat-happy (ConceptNode "Happy"))
(define chat-negative (ConceptNode "Negative"))
(StateLink chat-affect chat-happy)

(DefineLink
	(DefinedPredicate "chatbot is happy")
	(Equal
		(Set chat-happy)
		(Get (State chat-affect (Variable "$x")))))

(DefineLink
	(DefinedPredicate "chatbot is negative")
	(Equal
		(Set chat-negative)
		(Get (State chat-affect (Variable "$x")))))

; line 115 of behavior.cfg - time_to_change_face_target_min
(StateLink (SchemaNode "time_to_change_face_target_min") (NumberNode 8))
(StateLink (SchemaNode "time_to_change_face_target_max") (NumberNode 10))

(StateLink (SchemaNode "time_to_make_gesture_min") (NumberNode 6))
(StateLink (SchemaNode "time_to_make_gesture_max") (NumberNode 10))

(StateLink (SchemaNode "time_to_wake_up") (NumberNode 25))

(StateLink (SchemaNode "glance_probability") (NumberNode 0.7))
(StateLink (SchemaNode "sleep_probability") (NumberNode 0.1))
(StateLink (SchemaNode "wake_up_probability") (NumberNode 0.5))

;; The "look at neutral position" face. Used to tell the eye/head
;; movemet subsystem to move to a neutral position.
(define neutral-face (ConceptNode "0"))

; current_emotion_duration set to default_emotion_duration
; (StateLink (SchemaNode "current expression duration") (TimeNode 1.0)) ; in seconds
(StateLink (SchemaNode "current expression duration") (NumberNode 6.0)) ; in seconds

; --------------------------------------------------------
; temp scaffolding and junk.

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "Triggered: ~a \n" atom) (stv 1 1))

; --------------------------------------------------------
; Given the name of a emotion, pick one of the allowed emotional
; expressions at random. Example usage:
;
;   (cog-execute!
;      (PutLink (DefinedSchemaNode "Pick random expression")
;         (ConceptNode "positive")))
;
; This will pick out one of the "positive" emotions (defined above).
;
(DefineLink
	(DefinedSchema "Pick random expression")
	(LambdaLink
		(Variable "$emo")
		(RandomChoice
			(GetLink
				; Return a bunch of probability-expr pairs.
				(VariableList (Variable "$prob") (Variable "$expr"))
				(AndLink
					(Evaluation
						(Predicate "Emotion-expression")
						(ListLink (Variable "$emo") (Variable "$expr")))
					(State
						(ListLink
							(Variable "$emo")
							(Variable "$expr")
							(Schema "probability"))
						(Variable "$prob"))
				)))))

; As above, but for gestures.
(DefineLink
	(DefinedSchema "Pick random gesture")
	(LambdaLink
		(Variable "$emo")
		(RandomChoice
			(GetLink
				(VariableList (Variable "$prob") (Variable "$expr"))
				(AndLink
					(Evaluation
						(Predicate "Emotion-gesture")
						(ListLink (Variable "$emo") (Variable "$expr")))
					(State
						(ListLink
							(Variable "$emo")
							(Variable "$expr")
							(Schema "gest probability"))
						(Variable "$prob"))
				)))))

; Pick a random numeric value, lying in the range between min and max.
; The range min and max depends on an emotion-expression pair. For an
; example usage, see below.
(define (pick-value-in-range min-name max-name)
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$expr"))
		(RandomNumberLink
			(GetLink (VariableNode "$int-min")
				(StateLink (ListLink
					(VariableNode "$emo") (VariableNode "$expr")
					(SchemaNode min-name)) (VariableNode "$int-min")))
			(GetLink (VariableNode "$int-max")
				(StateLink (ListLink
					(VariableNode "$emo") (VariableNode "$expr")
					(SchemaNode max-name)) (VariableNode "$int-max")))
		)))

; Get a random intensity value for the indicated emotion-expression.
; That is, given an emotion-expression pair, this wil look up the
; min and max allowed intensity levels, and return a random number
; betwee these min and max values.
;
; Example usage:
;    (cog-execute!
;        (PutLink (DefinedSchemaNode "get random intensity")
;            (ListLink (ConceptNode "positive") (ConceptNode "engaged"))))
; will return an intensity level for the positive-egaged expression.
(DefineLink
	(DefinedSchemaNode "get random intensity")
	(pick-value-in-range "intensity-min" "intensity-max"))

; As above, but for gestures. Avoids a name-space collision.
(DefineLink
	(DefinedSchemaNode "get random gest intensity")
	(pick-value-in-range "gest intensity-min" "gest intensity-max"))

; Similar to above, but for duration. See explanation above.
(DefineLink
	(DefinedSchemaNode "get random duration")
	(pick-value-in-range "duration-min" "duration-max"))

(DefineLink
	(DefinedSchemaNode "get random repeat")
	(pick-value-in-range "repeat-min" "repeat-max"))

(DefineLink
	(DefinedSchemaNode "get random speed")
	(pick-value-in-range "speed-min" "speed-max"))

; Show a expression from a given emotional class. Sends the expression
; to ROS for display.  Sets a timestamp as well.  The intensity and
; duration of the expression is picked randomly from the parameters for
; the emotion-expression.
;
; Example usage:
;    (cog-evaluate!
;        (PutLink (DefinedPredicateNode "Show expression")
;           (ListLink (ConceptNode "positive") (ConceptNode "engaged"))))
;
(DefineLink
	(DefinedPredicateNode "Show expression")
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$expr"))
		(SequentialAndLink
			;; Record the time
			(TrueLink (DefinedSchemaNode "set expression timestamp"))
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicateNode "py:do_emotion")
				(ListLink
					(VariableNode "$expr")
					(PutLink
						(DefinedSchemaNode "get random duration")
						(ListLink (VariableNode "$emo") (VariableNode "$expr")))
					(PutLink
						(DefinedSchemaNode "get random intensity")
						(ListLink (VariableNode "$emo") (VariableNode "$expr")))
			))
		)
	))

; Show a gesture for a given emotional class. Sends the gesture
; to ROS for display.  The intensity, repetition and speed of the
; gesture is picked randomly from the parameters for the emotion-gesture.
;
; Example usage:
;    (cog-evaluate!
;        (PutLink (DefinedPredicateNode "Show gesture")
;           (ListLink (ConceptNode "positive") (ConceptNode "nod-1"))))
;
(DefineLink
	(DefinedPredicateNode "Show gesture")
	(LambdaLink
		(VariableList (VariableNode "$emo") (VariableNode "$gest"))
		(SequentialAndLink
			;; Send it off to ROS to actually do it.
			(EvaluationLink (GroundedPredicateNode "py:do_gesture")
				(ListLink
					(VariableNode "$gest")
					(PutLink
						(DefinedSchemaNode "get random gest intensity")
						(ListLink (VariableNode "$emo") (VariableNode "$gest")))
					(PutLink
						(DefinedSchemaNode "get random repeat")
						(ListLink (VariableNode "$emo") (VariableNode "$gest")))
					(PutLink
						(DefinedSchemaNode "get random speed")
						(ListLink (VariableNode "$emo") (VariableNode "$gest")))
			))
			;; Log the time.
			(TrueLink (PutLink
					(StateLink (SchemaNode "gesture-timestamp")
						(VariableNode "$x"))
					(TimeLink)))
		)
	))

;
; Pick an expression of the given, and send it to ROS for display.
; The expression is picked randomly from the class of expressions for
; the given emotion.  Likewise, the strength of display, and the
; duration are picked randomly.  The timestamp is recorded as well.
;
; Example usage:
;    (cog-evaluate!
;       (PutLink (DefinedPredicateNode "Show random expression")
;          (ConceptNode "positive")))
; will pick one of te "positive" emotions, and send it off.
;
;; line 305 -- pick_random_expression()
(DefineLink
	(DefinedPredicateNode "Show random expression")
	(LambdaLink
		(VariableNode "$emo")
		(PutLink
			(DefinedPredicateNode "Show expression")
			(ListLink
				(VariableNode "$emo")
				(PutLink
					(DefinedSchemaNode "Pick random expression")
					(VariableNode "$emo"))
			))
	))

;; Like the above, but for gestures
;; line 334 -- pick_random_gesture()
(DefineLink
	(DefinedPredicateNode "Show random gesture")
	(LambdaLink
		(VariableNode "$emo")
		(PutLink
			(DefinedPredicateNode "Show gesture")
			(ListLink
				(VariableNode "$emo")
				(PutLink
					(DefinedSchemaNode "Pick random gesture")
					(VariableNode "$emo"))
			))
	))

; --------------------------------------------------------
; Show facial expressions and gestures suitable for a given emotional
; state. These are radom selectors, picking some expression randomly
; from a meu of choices, ad displaying it.

;; Pick random expression, and display it.
(DefineLink
	(DefinedPredicateNode "Show positive expression")
	(PutLink (DefinedPredicateNode "Show random expression")
		(ConceptNode "positive")))

;; line 840 -- show_frustrated_expression()
(DefineLink
	(DefinedPredicateNode "Show frustrated expression")
	(PutLink (DefinedPredicateNode "Show random expression")
		(ConceptNode "frustrated")))

;; Pick random positive gesture
(DefineLink
	(DefinedPredicateNode "Pick random positive gesture")
	(PutLink (DefinedPredicateNode "Show random gesture")
		(ConceptNode "positive")))

; ------------------------------------------------------
; TODO --
;
; grep for NumberNode below, and make these (more easily) configurable.
; ------------------------------------------------------

; line 588 -- dice_roll("glance_new_face")
; XXX incomplete, needs refinement
(DefineLink
	(DefinedPredicateNode "dice-roll: glance new face")
	(GreaterThanLink
		(NumberNode "0.5")
		(RandomNumberLink (NumberNode 0) (NumberNode 1))))

(DefineLink
	(DefinedPredicateNode "dice-roll: glance lost face")
	(GreaterThanLink
		(NumberNode "0.5")
		(RandomNumberLink (NumberNode 0) (NumberNode 1))))

;; line 599 -- kwargs["event"] == "group_interaction"
(DefineLink
	(DefinedPredicateNode "dice-roll: group interaction")
	(GreaterThanLink
		(GetLink (StateLink (SchemaNode "glance_probability")
				(VariableNode "$x")))
		(RandomNumberLink (NumberNode 0) (NumberNode 1))))

;; line 609 -- kwargs["event"] == "go_to_sleep"
;; XXX incomplete, should depend on "bored-since" time
;; if bored for 5 minutes, go to sleep.
(DefineLink
	(DefinedPredicateNode "dice-roll: go to sleep")
	(GreaterThanLink
		(GetLink (StateLink (SchemaNode "sleep_probability")
				(VariableNode "$x")))
		(RandomNumberLink (NumberNode 0) (NumberNode 1))))

;; line 619 -- kwargs["event"] == "wake_up"
(DefineLink
	(DefinedPredicateNode "dice-roll: wake up")
	(GreaterThanLink
		(GetLink (StateLink (SchemaNode "wake_up_probability")
				(VariableNode "$x")))
		(RandomNumberLink (NumberNode 0) (NumberNode 1))))

; ------------------------------------------------------
; Basic utilities for working with newly-visible faces.

;; ------
;;
;; Return true if a new face has become visible.
;; A "new  face" is one that is visible (in the atomspace) but
;; has not yet been acked.
;; line 631, is_someone_arrived()
(DefineLink
	(DefinedPredicateNode "Did someone arrive?")
	(SatisfactionLink
		(AndLink
			; If someone is visible...
			(PresentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
			; but not yet acknowledged...
			(AbsentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
		)))

;; Return the set of newly-arrived faces.
(DefineLink
	(DefinedSchemaNode "New arrivals")
	(GetLink
		(AndLink
			; If someone is visible...
			(PresentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
			; but not yet acknowledged...
			(AbsentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
	)))

;; Return he person with whom we are currently interacting.
;; aka "current_face_target" in Owyl.
(DefineLink
	(DefinedSchemaNode "Current interaction target")
	(GetLink (StateLink interaction-state (VariableNode "$x"))))

;; Return true if some face has is no longer visible (has left the room)
;; We detect this by looking for "acked" faces tat are not also visible.
;; line 641, is_someone_left()
(DefineLink
	(DefinedPredicateNode "Did someone leave?")
	(SatisfactionLink
		(AndLink
			; If someone was previously acked...
			(PresentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
			; But is no loger visible...
			(AbsentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
		)))

;; Return list of recetly departed individuals
(DefineLink
	(DefinedSchemaNode "New departures")
	(GetLink
		(AndLink
			; If someone was previously acked...
			(PresentLink (EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id"))))
			; But is no loger visible...
			(AbsentLink (EvaluationLink (PredicateNode "visible face")
					(ListLink (VariableNode "$face-id"))))
	)))

;;
;; Was the the room empty, viz: Does the atomspace contains the link
;; (StateLink (AnchorNode "Room State") (ConceptNode "room empty"))?
;; Note that the room state is updated only when "Update room state"
;; is called, so faces may be visible, but the room marked as empty.
;; Think "level trigger" instead of "edge trigger".
;; line 665, were_no_people_in_the_scene
(DefineLink
	(DefinedPredicateNode "was room empty?")
	(EqualLink
		(SetLink room-empty)
		(GetLink (StateLink room-state (VariableNode "$x")))
	))

;; Is there someone present?  We check for acked faces.
;; The someone-arrived code converts newly-visible faces to acked faces.
;; line 683 is_face_target().
(DefineLink
	(DefinedPredicateNode "Someone visible")
	(SatisfactionLink
		(PresentLink
			(EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id")))
		)))

;; Return the number of visible faces
;; line 690 -- is_more_than_one_face_target()
(DefineLink
	(DefinedSchemaNode "Num visible faces")
	(ArityLink
		(GetLink (EvaluationLink (PredicateNode "acked face")
			(ListLink (VariableNode "$face-id"))))))

; True if more than one face is visible.
(DefineLink
	(DefinedPredicateNode "More than one face visible")
	(GreaterThanLink
		(DefinedSchemaNode "Num visible faces")
		(NumberNode 1)))


;; Randomly select a face out of the crowd.
;; line 747 -- select_a_face_target() and also
(DefineLink
	(DefinedSchemaNode "Select random face")
	(RandomChoiceLink (GetLink
		(EvaluationLink (PredicateNode "acked face")
			(ListLink (VariableNode "$face-id")))
	)))

;; line 752 -- select_a_glance_target()
(DefineLink
	(DefinedPredicateNode "Select random glance target")
	(SequentialAndLink
		; Recursive loop, keep picking, while the current glance target
		; is the same as the current interaction target.
		(TrueLink
			(PutLink (StateLink glance-state (VariableNode "$face-id"))
				(DefinedSchemaNode "Select random face")))
		(EqualLink
			(GetLink (StateLink glance-state (VariableNode "$face-id")))
			(GetLink (StateLink interaction-state (VariableNode "$face-id")))
		)
		(DefinedPredicateNode "More than one face visible")
		(DefinedPredicateNode "Select random glance target")
	))

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
	))

;; Update the room empty/full status; update the list of acknowledged
;; faces.
;; line 973 -- clear_new_face_target()
(DefineLink
	(DefinedPredicateNode "Update status")
	(SequentialAndLink
		(DefinedPredicateNode "Update room state")
		(TrueLink (PutLink
				(EvaluationLink (PredicateNode "acked face")
						(ListLink (VariableNode "$face-id")))
				(DefinedSchemaNode "New arrivals")))
	))

;; Remove the lost faces from "acked face" (so that "acked face" accurately
;; reflects the visible faces)
;; line 980 -- clear_lost_face_target()
(DefineLink
	(DefinedPredicateNode "Clear lost face")
	(TrueLink (PutLink
		(DeleteLink
			(EvaluationLink (PredicateNode "acked face")
				(ListLink (VariableNode "$face-id"))))
		(DefinedSchemaNode "New departures"))
	))
;; ------
;;
;; Return true if interacting with someone.
;; line 650, is_interacting_with_someone
;; (cog-evaluate! (DefinedPredicateNode "is interacting with someone?"))
(DefineLink
	(DefinedPredicateNode "is interacting with someone?")
	(NotLink (EqualLink
		(SetLink no-interaction)
		(GetLink (StateLink interaction-state (VariableNode "$x"))))
	))

;; Send ROS message to look at the person we are interacting with.
;; line 742, assign_face_target
(DefineLink
	(DefinedSchemaNode "look at person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:look_at_face")
			(ListLink (VariableNode "$face")))
		(GetLink (StateLink interaction-state (VariableNode "$x")))
	))

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
	(DefinedSchemaNode "glance at new person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New arrivals")
	))

;; line 827 -- glance_at_lost_face()
(DefineLink
	(DefinedSchemaNode "glance at lost face")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New departures")))

;; Move to a neutral head position. Right now, this just issues a
;; look-at command; it could do more (e.g. halt the chatbot.)
;; line 845, return_to_neutral_position
(DefineLink
	(DefinedPredicateNode "return to neutral")
	(SequentialAndLink
		(EvaluationLink (GroundedPredicateNode "py:look_at_face")
			(ListLink neutral-face))
	))

; ------------------------------------------------------
; Time-stamp-related stuff.

;; Set a timestamp. XXX todo replace this with timeserver.
;; line 757, record_start_time
(DefineLink
	(DefinedSchemaNode "set interaction timestamp")
	(PutLink
		(StateLink (SchemaNode "start-interaction-timestamp")
			(VariableNode "$x"))
		(TimeLink)))

(DefineLink
	(DefinedSchemaNode "get interaction timestamp")
	(GetLink
		(StateLink (SchemaNode "start-interaction-timestamp")
			(VariableNode "$x"))))

(DefineLink
	(DefinedSchemaNode "set expression timestamp")
	(PutLink
		(StateLink (SchemaNode "start-expression-timestamp")
			(VariableNode "$x"))
		(TimeLink)))

(DefineLink
	(DefinedSchemaNode "get expression timestamp")
	(GetLink
		(StateLink (SchemaNode "start-expression-timestamp")
			(VariableNode "$x"))))

(DefineLink
	(DefinedSchemaNode "set bored timestamp")
	(PutLink
		(StateLink (SchemaNode "start-boredom-timestamp")
			(VariableNode "$x"))
		(TimeLink)))

(DefineLink
	(DefinedSchemaNode "set sleep timestamp")
	(PutLink
		(StateLink (SchemaNode "start-sleep-timestamp")
			(VariableNode "$x"))
		(TimeLink)))

(DefineLink
	(DefinedSchemaNode "get sleep timestamp")
	(GetLink
		(StateLink (SchemaNode "start-sleep-timestamp")
			(VariableNode "$x"))))

;; Evaluate to true, if an expression should be shown.
;; line 933, should_show_expression()
(DefineLink
	(DefinedPredicateNode "Time to change expression")
	(GreaterThanLink
		(MinusLink
			(TimeLink)
			(DefinedSchemaNode "get expression timestamp"))
		(GetLink (StateLink (SchemaNode "current expression duration")
			(VariableNode "$x"))) ; in seconds
	))

(DefineLink
	(DefinedPredicateNode "Time to make gesture")
	(GreaterThanLink
		; Minus lik computes number of seconds since interaction start.
		(MinusLink
			(TimeLink)
			(GetLink
				(StateLink (SchemaNode "gesture-timestamp")
					(VariableNode "$x"))))
		; Random number in the configured range.
		(RandomNumberLink
			(GetLink (StateLink (SchemaNode "time_to_make_gesture_min")
				(VariableNode "$min")))
			(GetLink (StateLink (SchemaNode "time_to_make_gesture_max")
				(VariableNode "$max"))))
	))


; Return true if it is time to interact with someone else.
;; line 697 -- is_time_to_change_face_target()
(DefineLink
	(DefinedPredicateNode "Time to change interaction")
	(GreaterThanLink
		; Minus lik computes number of seconds since interaction start.
		(MinusLink
			(TimeLink)
			(DefinedSchemaNode "get interaction timestamp"))
		; Random number in the configured range.
		(RandomNumberLink
			(GetLink (StateLink (SchemaNode "time_to_change_face_target_min")
				(VariableNode "$min")))
			(GetLink (StateLink (SchemaNode "time_to_change_face_target_max")
				(VariableNode "$max"))))
	))

; Return true if we've been sleeping for long enough (i.e. longer than
; the time_to_wake_up parameter.)
; line 707 -- is_time_to_wake_up()
(DefineLink
	(DefinedPredicateNode "Time to wake up")
	(GreaterThanLink
		(MinusLink
			(TimeLink)
			(DefinedSchemaNode "get sleep timestamp"))
		(GetLink (StateLink (SchemaNode "time_to_wake_up")
			(VariableNode "$x"))) ; in seconds
	))

; ------------------------------------------------------
; More complex interaction sequences.

;; Interact with the curent face target.
;; line 762, interact_with_face_target()
;; XXX Needs to be replaced by OpenPsi emotional state modelling.
;; XXX Almost a complete implementation of whats in owyl...  but
;; XXX The owyl pick_instant code is insane...
(DefineLink
	(DefinedPredicateNode "Interact with face")
	(SequentialAndLink
		;; Look at the interaction face - line 765
		(TrueLink (PutLink
			(EvaluationLink (GroundedPredicateNode "py:look_at_face")
				(ListLink (VariableNode "$face")))
			(GetLink (StateLink interaction-state (VariableNode "$x")))))

		;; Show random expressions only if NOT talking
		; aka "do_pub_emotions=False" in the new owyl tree.
		(DefinedPredicate "chatbot is listening")

		;; line 768
		(SequentialOrLink
			(NotLink (DefinedPredicateNode "Time to change expression"))
			(DefinedPredicateNode "Show positive expression")
		)
		(SequentialOrLink
			(NotLink (DefinedPredicateNode "Time to make gesture"))
			(DefinedPredicateNode "Pick random positive gesture"))
	))

; ------------------------------------------------------
;; Sequence - if there were no people in the room, then look at the
;; new arrival.
;; line 391 -- owyl.sequence
;; (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
(DefineLink
	(DefinedPredicateNode "Was Empty Sequence")
	(SequentialAndLink
		;; line 392
		(DefinedPredicateNode "was room empty?")
		(TrueLink (DefinedSchemaNode "interact with new person"))
		(TrueLink (DefinedSchemaNode "look at person"))
		(TrueLink (DefinedSchemaNode "set interaction timestamp"))
		(PutLink (DefinedPredicateNode "Show random expression")
			(ConceptNode "new-arrival"))
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Look at newly arrived person")))
	))

(DefineLink
	(DefinedSchemaNode "interact with new person")
	(PutLink (StateLink interaction-state (VariableNode "$x"))
		(DefinedSchemaNode "New arrivals")))

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
(DefineLink
	(DefinedPredicateNode "Respond to new arrival")
	(SequentialOrLink
		(DefinedPredicateNode "Was Empty Sequence")
		(DefinedPredicateNode "Interacting Sequence")
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Ignoring new person"))) ; line 406
			(TrueLink)
	))

;; Check to see if a new face has become visible.
;; line 386 -- someone_arrived()
(DefineLink
	(DefinedPredicateNode "New arrival sequence")
	(SequentialAndLink
		(DefinedPredicateNode "Did someone arrive?")
		(DefinedPredicateNode "Respond to new arrival")
		(DefinedPredicateNode "Update status")
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
		(DefinedPredicateNode "Update status")
	))

;; Interact with people
;; line 457 -- interact_with_people()
(DefineLink
	(DefinedPredicateNode "Interact with people")
	(SequentialAndLink ; line 458
		; True, if there is anyone visible.
		(DefinedPredicateNode "Someone visible") ; line 459
		; This or-link is true if we're not interacting with anyone,
		; or if there are several people and its time to change up.
		(SequentialOrLink ; line 460
			; ##### Start A New Interaction #####
			(SequentialAndLink ; line 462
				(SequentialOrLink ; line 463
					(NotLink (DefinedPredicateNode "is interacting with someone?"))
					(SequentialAndLink ; line 465
						(DefinedPredicateNode "More than one face visible")
						(DefinedPredicateNode "Time to change interaction")))
				; Select a new face target
				(DefinedPredicateNode "Start new interaction")
				(DefinedPredicateNode "Interact with face"))

			; ##### Glance At Other Faces & Continue With The Last Interaction
			(SequentialAndLink ; line 476
				(EvaluationLink (GroundedPredicateNode "scm: print-msg")
					(ListLink (Node "--- Continue interaction")))
				(SequentialOrLink  ; line 478
					(SequentialAndLink ; line 479
						(DefinedPredicateNode "More than one face visible")
						(DefinedPredicateNode "dice-roll: group interaction")
						(DefinedPredicateNode "glance at random face"))
					(TrueLink)) ; line 485
				(DefinedPredicateNode "Interact with face")
				(SequentialOrLink  ; line 488
					(SequentialAndLink ; line 489
; XXX incomplete!  need the face study saccade stuff...
						(FalseLink)
					)
					(TrueLink))  ; line 493
			))
	))

; ------------------------------------------------------
; Empty-room behaviors. We either search for attention, or we sleep,
; or we wake up.

; line 898 -- search_for_attention.
; XXX not done.
(DefineLink
	(DefinedPredicateNode "Search for attention")
	(SequentialAndLink
	))

; Call once, to fall asleep.
; line 941 -- go_to_sleep
(DefineLink
	(DefinedPredicateNode "Go to sleep")
	(SequentialAndLink
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Go to sleep.")))
		(TrueLink (DefinedSchemaNode "set sleep timestamp"))
		(PutLink (DefinedPredicateNode "Show random expression")
			(ConceptNode "sleep"))
		(PutLink (DefinedPredicateNode "Show random gesture")
			(ConceptNode "sleep"))
		(TrueLink (PutLink (StateLink soma-state (VariableNode "$x"))
			(SetLink soma-sleeping)))
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
	(DefinedPredicateNode "Wake up")
	(SequentialAndLink
		(EvaluationLink (GroundedPredicateNode "scm: print-msg")
			(ListLink (Node "--- Wake up!")))
		(TrueLink (DefinedSchemaNode "set bored timestamp"))
		(TrueLink (PutLink (StateLink soma-state (VariableNode "$x"))
			soma-awake))
		(PutLink (DefinedPredicateNode "Show random expression")
			(ConceptNode "wake-up"))
		(PutLink (DefinedPredicateNode "Show random gesture")
			(ConceptNode "wake-up"))
	))

;; Nothing is happening (no faces are visibile)
;; Go to sleep after a while, and wake up every now and then.
;; line 507 -- nothing_is_happening()
(DefineLink
	(DefinedPredicateNode "Nothing is happening")
	(SequentialAndLink  ; line 508
		(SequentialOrLink  ; line 509
			; ##### Is Not Sleeping #####
			(SequentialAndLink ; line 511
				(NotLink (EqualLink
					(SetLink soma-sleeping)
					(GetLink (StateLink soma-state (VariableNode "$x")))))

				(SequentialOrLink  ; line 513
					; ##### Go To Sleep #####
					(SequentialAndLink  ; line 515
						(DefinedPredicateNode "dice-roll: go to sleep")
						(DefinedPredicateNode "Go to sleep"))
					; ##### Search For Attention #####
					; If we didn't fall asleep above, then search for attention.
					(DefinedPredicateNode "Search for attention")
				))
			; ##### Is Sleeping #####
			(SequentialOrLink  ; line 528
				; ##### Wake Up #####
				(SequentialAndLink  ; line 530
					(DefinedPredicateNode "dice-roll: wake up")
					; did we sleep for long enough?
					(DefinedPredicateNode "Time to wake up")
					(DefinedPredicateNode "Wake up")
				)
				; ##### Continue To Sleep #####
				(DefinedPredicateNode "Continue sleeping")
			)
		)

		; ##### If Interruption && Sleeping -> Wake Up #####
		;; XXX This never runs, since the face-detected code is
		;; triggered before we can get to here. Thus, the wake-up
		;; sequence never runs, and the soma-state is incorrect...
		(SequentialAndLink  ; line 545
			(EqualLink (SetLink soma-sleeping)
				(GetLink (StateLink soma-state (VariableNode "$x"))))
			(DefinedPredicateNode "Did someone arrive?")
			(DefinedPredicateNode "Wake up")
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
		; ... and show a random gesture from "listening" set.
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "listening"))
		; ... and also, sometimes, the "chatbot_positive_nod"
		(Put (DefinedPredicate "Show random gesture")
			(ConceptNode "chat-positive-nod"))
		; ... and switch state to "talking"
		(True (Put (State chat-state (Variable "$x")) chat-talk))
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
				(Put (DefinedPredicateNode "Show random expression")
					(ConceptNode "neutral-speech"))
				; ... nod slowly ...
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
			)
			(SequentialAnd
				; If chatbot is not happy ...
				(DefinedPredicate "chatbot is negative")
				; ... show one of the frustrated expressions
				(Put (DefinedPredicateNode "Show random expression")
					(ConceptNode "frustrated"))
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

;; Main loop. Uses tail recursion optimizatio to form the loop.
;; line 556 -- build_tree()
(DefineLink
	(DefinedPredicate "main loop")
	(SatisfactionLink
		(SequentialAnd
			(SequentialOr
				(DefinedPredicate "New arrival sequence")
				(DefinedPredicate "Someone left")
				(DefinedPredicate "Interact with people")
				(DefinedPredicate "Nothing is happening")
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

;
; Silence the output.
(TrueLink)

;; Actually set it running
(all-threads)
(run)

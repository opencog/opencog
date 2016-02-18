;
; self-model.scm
;
; Model of Eva's current physical state, represented in the Atomspace.
;
; This attempts to maintain a model, in the atomspace, of what the robot
; is actually doing, from moment to moment.  This model is important for
; multiple reasons:
;
; -- It is needed for Action Orchestration, to make sure that multiple
;    conflicting command sources do not cause the robot to do incoherent
;    things (such as smile and frown at the same time, or move lips
;    without talking, etc.)
;
; -- It is needed for self-awareness, so that the chatbot can respond to
;    questions about what Eva is doing.
;
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))

(load-from-path "utilities.scm")

; (display %load-path)
(add-to-load-path "../src")
(load-from-path "faces.scm")
(load-from-path "cfg-tools.scm")
(load-from-path "behavior-cfg.scm")

; (use-modules (opencog logger))
; (cog-logger-set-stdout #t)

; ------------------------------------------------------
; State variables

; Soma: awake, agitated, excited, tired, manic, depressed
(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))
(define soma-awake (ConceptNode "Awake"))
(define soma-bored (ConceptNode "Bored"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; Currently, interaction-state will be linked to the face-id of
;; person with whom interaction is taking place. (current_face_target in owyl)
(define interaction-state (AnchorNode "Interaction State"))
(define no-interaction (ConceptNode "none"))

(StateLink interaction-state no-interaction)

; The face to glance at.
(define glance-state (AnchorNode "Glance State"))
(StateLink glance-state no-interaction)

;; Linked to face-id that needs immediate interaction.
;; Currently it is set from ROS
(define request-interaction-state (AnchorNode "Request Interaction"))
(StateLink request-interaction-state no-interaction)

;; The "look at neutral position" face. Used to tell the eye/head
;; movemet subsystem to move to a neutral position.
(define neutral-face (ConceptNode "0"))

; --------------------------------------------------------
; Chatbot-related stuff.  In the curent design, the chatbot talks
; whenever it feels like it; we are simply told when it is talking
; when it has stopped talking, and what emotions we should display,
; so that it's consistent with the speech emotions.

; Chat state. Is the robot talking, or not, right now?
; NB the python code uses these defines!
(define chat-state (AnchorNode "Chat State"))
(define chat-listen (ConceptNode "Listening"))
(define chat-talk   (ConceptNode "Talking"))
(define chat-start  (ConceptNode "Start Talking"))
(define chat-stop   (ConceptNode "Stop Talking"))
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

; --------------------------------------------------------
; Time-stamp-related stuff.

;; Define setters and getters for timestamps. Perhaps this should
;; be replaced by the timeserver??
;; line 757, record_start_time

(define (timestamp-template name)

	; The name of state node holding the timestamp.
	(define ts-name (string-append "start-" name "-timestamp"))
	(define prev-ts (string-append "previous-" name "-call"))

	; The state node actually holding the timestamp.
	(State (Schema ts-name) (Number 0))

	; timestamp setter
	(DefineLink
		(DefinedSchema (string-append "set " name " timestamp"))
		(Put (State (Schema ts-name) (Variable "$x")) (TimeLink)))

	; timestamp getter
	(DefineLink
		(DefinedSchema (string-append "get " name " timestamp"))
		(Get (State (Schema ts-name) (Variable "$x"))))

	; Additional state, used for computing integral, for probabilities.
	; See below, in the time-to-change template.
	(State (Schema prev-ts) (Number 0))
)

; "interaction" -- record the start time of an interaction.
; defines (DefinedSchema "set-interaction-timestamp") etc.
(timestamp-template "interaction")

; "expression" -- time when a new expression started being shown.
(timestamp-template "expression")
; "gesture" -- time when the last gesture was made.
(timestamp-template "gesture")

; "bored" -- when Eva last got bored.
(timestamp-template "bored")
; "sleep" -- when Eva fell asleep.
(timestamp-template "sleep")

; "attn-search" -- when Eva started searching for attention.
(timestamp-template "attn-search")

; Define a predicate that evaluates to true or false, if it is time
; to do something. PRED-NAME is the name given to the predicate,
; TS-NAME is the name given to the timestamp that holds the start-time;
; MIN-NAME and MAX-NAME are the string names of the configurable
; min and max bounds for the time interval.  So, if the elapsed time
; since the timestamp is less than MIN, then return false; if the
; elapsed time is greater than MAX then return true; else return a
; with increasing random liklihood.
;
; To compute the likelihood correctly, we have to compute the
; integral since the last time that this check was made. Thus,
; if we are calling this predicate 10 times a second, the probability
; of a transition will be failry small; but if we call it once a second,
; the probability will be ten times greater... computing even this
; simple integral in atomese is painful. Yuck. But we have to do it.
(define (change-template pred-name ts-name min-name max-name)
	(define get-ts (string-append "get " ts-name " timestamp"))
	(define prev-ts (string-append "previous-" ts-name "-call"))
	(define delta-ts (string-append "delta-" ts-name "-time"))
	(DefineLink
		(DefinedPredicate pred-name)
		(SequentialOr
			; If elapsed time greater than max, then true.
			(GreaterThan
				; Minus computes number of seconds since interaction start.
				(Minus (TimeLink) (DefinedSchema get-ts))
				(Get (State (Schema max-name) (Variable "$max")))
			)
			(SequentialAnd
				; Delta is the time since the last check.
				(True (Put (State (Schema delta-ts) (Variable "$x"))
						(Minus (TimeLink)
							(Get (State (Schema prev-ts) (Variable "$p"))))))
				; Update time of last check to now. Must record this
				; timestamp before the min-time rejection, below.
				(True (Put (State (Schema prev-ts) (Variable "$x")) (TimeLink)))

				; If elapsed time less than min, then false.
				(GreaterThan
					; Minus computes number of seconds since interaction start.
					(Minus (TimeLink) (DefinedSchema get-ts))
					(Get (State (Schema min-name) (Variable "$min")))
				)

				; Compute integral: how long since last check?
				; Perform a pro-rated coin flip. If it is only a very short
				; time since we were last called, it is very unlikely that
				; well the random number will come up heads.  But if its
				; been a long time, then very likely the coin will come up
				; heads.
				(GreaterThan
					(Get (State (Schema delta-ts) (Variable "$delta")))
					; Random number in the configured range.
					(RandomNumber
						(Number 0)
						(Minus
							(Get (State (Schema max-name) (Variable "$max")))
							(Get (State (Schema min-name) (Variable "$min")))))
				)
			)
	)))

; Return true if it is time to interact with someone else.
;; line 697 -- is_time_to_change_face_target()
(change-template "Time to change interaction" "interaction"
	"time_to_change_face_target_min" "time_to_change_face_target_max")

; Return true if we've been sleeping for long enough (i.e. longer than
; the time_to_wake_up parameter).
; line 707 -- is_time_to_wake_up()
(change-template "Time to wake up" "sleep"
	"time_sleeping_min" "time_sleeping_max")

; Return true if we've been bored for a long time (i.e. longer than
; the time_bored_to_sleep parameter).
; line 611 -- bored_since, sleep probability.
(change-template "Bored too long" "bored"
	"time_boredom_min" "time_boredom_max")

;; Evaluate to true, if an expression should be shown.
;; (if it is OK to show a new expression). Prevents system from
;; showing new facial expressions too frequently.
;; line 933, should_show_expression()
(change-template "Time to change expression" "expression"
	"time_since_last_expr_min" "time_since_last_expr_max")

(change-template "Time to make gesture" "gesture"
	"time_since_last_gesture_min" "time_since_last_gesture_max")

(change-template "Time to change gaze" "attn-search"
	"time_search_attn_min" "time_search_attn_max")

; --------------------------------------------------------
; Some debug prints.

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "~a\n" atom) (stv 1 1))

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
			(True (DefinedSchema "set gesture timestamp"))
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
; will pick one of the "positive" emotions, and send it off to ROS.
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
;; Will return a SetLink holding zero, one or more face id's
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

;; Return the person with whom we are currently interacting.
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

;; Update the room empty/full status; update the list of acknowledged
;; faces.
;; line 973 -- clear_new_face_target()
(DefineLink
	(DefinedPredicate "Update status")
	(SequentialAnd
		(DefinedPredicate "Update room state")
		(True (Put
				(Evaluation (Predicate "acked face")
						(ListLink (Variable "$face-id")))
				; If more than one new arrival, pick one randomly.
				(RandomChoice (DefinedSchema "New arrivals"))))
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
	(DefinedPredicate "is interacting with someone?")
	(NotLink (Equal
		(SetLink no-interaction)
		(Get (State interaction-state (Variable "$x"))))
	))


;; Return true if someone requests interaction.  This person will
;; become the new focus of attention.
(DefineLink
	(DefinedPredicate "Someone requests interaction?")
	(NotLink (Equal
		(SetLink no-interaction)
		(Get (State request-interaction-state (Variable "$x"))))
	))


;; Send ROS message to look at the person we are interacting with.
;; line 742, assign_face_target
(DefineLink
	(DefinedSchema "look at person")
	(Put
		(Evaluation (GroundedPredicate "py:look_at_face")
			(ListLink (Variable "$face")))
		(Get (State interaction-state (Variable "$x")))
	))

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

(DefineLink
	(DefinedSchema "interact with new person")
	(Put (State interaction-state (Variable "$x"))
		; If more than one new arrival, pick one randomly.
		(RandomChoice (DefinedSchema "New arrivals"))))

;; Set current-interaction face to the requested face
(DefineLink
	(DefinedSchema "interact with requested person")
	(Put (State interaction-state (Variable "$face-id"))
		(Get (State request-interaction-state (Variable "$x")))))

(DefineLink
	(DefinedSchema "clear requested face")
	(Put (State request-interaction-state (Variable "$face-id"))
		no-interaction))

;; ------------------------------------------------------------------

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

(use-modules (opencog logger))
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
		(Put (State (Schema ts-name) (Variable "$x")) (Time)))

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
				(Minus (Time) (DefinedSchema get-ts))
				(Get (State (Schema max-name) (Variable "$max")))
			)
			(SequentialAnd
				; Delta is the time since the last check.
				(True (Put (State (Schema delta-ts) (Variable "$x"))
						(Minus (Time)
							(Get (State (Schema prev-ts) (Variable "$p"))))))
				; Update time of last check to now. Must record this
				; timestamp before the min-time rejection, below.
				(True (Put (State (Schema prev-ts) (Variable "$x")) (Time)))

				; If elapsed time less than min, then false.
				(GreaterThan
					; Minus computes number of seconds since interaction start.
					(Minus (Time) (DefinedSchema get-ts))
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
; XXX incomplete!  need the face study saccade stuff...
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
				(Minus (Time) (DefinedSchema "get bored timestamp"))))
		(True (DefinedSchema "set sleep timestamp"))
		(Put (DefinedPredicate "Show random expression")
			(Concept "sleep"))
		(Put (DefinedPredicate "Show random gesture")
			(Concept "sleep"))
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
				(Minus (Time) (DefinedSchema "get sleep timestamp"))))
		(TrueLink (DefinedSchema "set bored timestamp"))

		; Change soma state to being awake.
		(True (Put (State soma-state (Variable "$x")) soma-awake))
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

;; Main loop. Uses tail recursion optimizatio to form the loop.
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

;
; Silence the output.
(TrueLink)

;; Actually set it running
(all-threads)
(run)

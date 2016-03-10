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
; Example usage:
; Load the needed modules.
; (use-modules (opencog) (opencog query) (opencog exec))
; (use-modules (opencog atom-types) (opencog python))
; (use-modules (opencog eva-model))
;;;; start roscore before doing the load below.
; (python-eval "execfile('atomic.py')")
;
; Examples and debugging hints:
; Some (but not all) state queries:
; (cog-evaluate! (DefinedPredicate "chatbot is talking"))
; (cog-evaluate! (DefinedPredicate "chatbot is listening"))
; (cog-evaluate! (DefinedPredicate "chatbot is happy"))
; (cog-evaluate! (DefinedPredicateNode "Did someone arrive?"))
; (cog-evaluate! (DefinedPredicateNode "Someone visible"))
; (cog-execute! (DefinedSchemaNode "Num visible faces"))
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))

; XXX the below does not really belong here; where does it belong?
(use-modules (opencog nlp chatbot-eva)) ; Needed for process-query

; ------------------------------------------------------
; State variables
; XXX FIXME There are a bunch of define-publics in here, they probably
; should not be; they're needed only by the behavior module.

; Soma: awake, agitated, excited, tired, manic, depressed
(define-public soma-state (AnchorNode "Soma State"))
(define-public soma-sleeping (ConceptNode "Sleeping"))
(define-public soma-awake (ConceptNode "Awake"))
(define-public soma-bored (ConceptNode "Bored"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; True if sleeping, else false.
(DefineLink
	(DefinedPredicate "Is sleeping?")
	(Equal (SetLink soma-sleeping)
		(Get (State soma-state (Variable "$x")))))

;; True if bored, else false
(DefineLink
	(DefinedPredicate "Is bored?")
	(Equal (SetLink soma-bored)
		(Get (State soma-state (Variable "$x")))))

; -----------
; The "emotional state" of the robot.  Corresponds to states declared
; in the `cfg-*.scm` file.
(define-public emotion-state (AnchorNode "Emotion State"))
(define-public emotion-neutral (ConceptNode "neutral"))

(StateLink emotion-state emotion-neutral)

; -----------
;; The eye-contact-state will be linked to the face-id of
;; person with whom we are making eye-contact with.
(define-public eye-contact-state (AnchorNode "Eye Contact State"))
(define-public no-interaction (ConceptNode "none"))

(StateLink eye-contact-state no-interaction)

; The face to glance at.
(define-public glance-state (AnchorNode "Glance State"))
(StateLink glance-state no-interaction)

;; Linked to face-id that needs immediate interaction.
;; Currently it is set from ROS
(define request-eye-contact-state (AnchorNode "Request Interaction"))
(StateLink request-eye-contact-state no-interaction)

;; The "look at neutral position" face. Used to tell the eye/head
;; movemet subsystem to move to a neutral position.
(define neutral-face (ConceptNode "0"))

; --------------------------------------------------------
; Chatbot-related stuff.  In the curent design, the chatbot talks
; whenever it feels like it; we are simply told when it is talking
; when it has stopped talking, and what emotions we should display,
; so that it's consistent with the speech emotions.

; Chat state. Is the robot talking (vocalizing), or not, right now?
; NB the python code in put_atoms.py uses these defines!
; This is a state-machine, valid transitions are:
; listening -> started talking
; started talking -> talking
; talking -> stoped talking
; stopped talking -> listening.
(define-public chat-state (AnchorNode "Chat State"))
(define-public chat-listen (ConceptNode "Listening"))
(define-public chat-start  (ConceptNode "Start Talking"))
(define-public chat-talk   (ConceptNode "Talking"))
(define-public chat-stop   (ConceptNode "Stop Talking"))
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
; XXX FIXME: Note also: we currently fail to distinguish the affect
; that was perceived, from our own state. There is a ROS message that
; informs us about what the perceived affect was: it sets this state.
;
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
; Speech-to-text (STT) relaed stuff.
; If the STT system hears soemthing, it sends us the text string.
; Handle it here.

; XXX the below does not really belong here; where does it belong?
; Pass the text that STT heard into the OpenCog chatbot.
; XXX procees-query is not really the best API, here.
; Must run in a new thread, else it deadlocks in python,
; since the text processing results in python calls.
(define-public (dispatch-text txt)
	(call-with-new-thread
		(lambda () (process-query "luser" (cog-name txt)))
	)
	(stv 1 1)
)

(DefineLink
	(DefinedPredicate "heard text")
	(LambdaLink
		(Variable "$text")
		(SequentialAnd
			(Evaluation (GroundedPredicate "scm: dispatch-text")
				(ListLink (Variable "$text")))

			; Set timestamp for when something was last heard.
			(TrueLink (DefinedSchemaNode "set heard-something timestamp"))
		)
	)
)


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
; defines (DefinedSchema "set interaction timestamp") etc.
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

; "heard-something" -- when Eva heard a sentence from STT.
(timestamp-template "heard-something")

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
	(GetLink (StateLink eye-contact-state (VariableNode "$x"))))

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
			(GetLink (StateLink eye-contact-state (VariableNode "$face-id")))
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
;; This is a compound predicate: we are interacting if the interaction
;; state is set, or if the TTS system/chatbot is still vocalizing.
;; (cog-evaluate! (DefinedPredicateNode "is interacting with someone?"))
(DefineLink
	(DefinedPredicate "is interacting with someone?")
	(OrLink
		; true if talking not listening.
		(NotLink (DefinedPredicate "chatbot is listening"))
		; true if not not-interacting.
		(NotLink (Equal
			(SetLink no-interaction)
			(Get (State eye-contact-state (Variable "$x"))))
	)))


;; Return true if someone requests interaction.  This person will
;; become the new focus of attention.
(DefineLink
	(DefinedPredicate "Someone requests interaction?")
	(NotLink (Equal
		(SetLink no-interaction)
		(Get (State request-eye-contact-state (Variable "$x"))))
	))


;; Send ROS message to look at the person we are interacting with.
(DefineLink
	(DefinedSchema "look at person")
	(Put
		(Evaluation (GroundedPredicate "py:look_at_face")
			(ListLink (Variable "$face")))
		(Get (State eye-contact-state (Variable "$x")))
	))

;; Move to a neutral head position. Right now, this just issues a
;; look-at command; it could do more (e.g. halt the chatbot.)
(DefineLink
	(DefinedPredicateNode "return to neutral")
	(SequentialAndLink
		(EvaluationLink (GroundedPredicateNode "py:look_at_face")
			(ListLink neutral-face))
	))

; ------------------------------------------------------

; Start interacting with a new face picked randomly from the crowd.
(DefineLink
	(DefinedPredicateNode "Start new interaction")
	(SequentialAndLink
		; First, pick a face at random...
		(TrueLink (PutLink
			(StateLink eye-contact-state (VariableNode "$face-id"))
			(DefinedSchemaNode "Select random face")))
		; Record a timestamp
		(TrueLink (DefinedSchemaNode "set interaction timestamp"))
		; Diagnostic print
		(Evaluation (GroundedPredicate "scm: print-msg-face")
			(ListLink (Node "--- Start new interaction")))
	))

(DefineLink
	(DefinedPredicate "interact with new person")
	(SequentialAnd
		(True (Put (State eye-contact-state (Variable "$x"))
			; If more than one new arrival, pick one randomly.
			(RandomChoice (DefinedSchema "New arrivals"))))
		(TrueLink (DefinedSchema "set interaction timestamp"))
	))

;; Set current-interaction face to the requested face
(DefineLink
	(DefinedPredicate "interact with requested person")
	(SequentialAnd
		(True (Put (State eye-contact-state (Variable "$face-id"))
			(Get (State request-eye-contact-state (Variable "$x")))))
		(TrueLink (DefinedSchema "set interaction timestamp"))
	))

(DefineLink
	(DefinedSchema "clear requested face")
	(Put (State request-eye-contact-state (Variable "$face-id"))
		no-interaction))

;; ------------------------------------------------------------------

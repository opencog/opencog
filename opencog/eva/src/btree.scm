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
; etc: (remove-face id)  (show-room-state) (show-interaction-state)
; (show-visible-faces)
;
; Unit test the new-arrival sequence:
; (make-new-face "42")
; (cog-evaluate! (DefinedPredicateNode "New arrival sequence"))
; (show-acked-faces)
; (show-room-state)
; (show-interaction-state)
; (cog-evaluate! (DefinedPredicateNode "Interact with people"))
;

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog) (opencog query) (opencog exec))
(use-modules (opencog atom-types))
(use-modules (opencog cogserver))
(start-cogserver "../scripts/opencog.conf")

; (system "echo \"py\\n\" | cat - atomic.py |netcat localhost 17020")
(system "echo \"py\\n\" | cat - atomic-dbg.py |netcat localhost 17020")

(load-from-path "utilities.scm")

; (display %load-path)
; (add-to-load-path "../src")
(load-from-path "faces.scm")
(load-from-path "behavior-cfg.scm")

(use-modules (opencog logger))
; (cog-logger-set-stdout #t)


; ------------------------------------------------------
; State variables

(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; Currently, interaction-state will be linked to the face-id of
;; person with whom interaction is taking place. (current_face_target in owyl)
(define interaction-state (AnchorNode "Interaction State"))
(define no-interaction (ConceptNode "none"))

(StateLink interaction-state no-interaction)
(StateLink (SchemaNode "start-interaction-timestamp") (NumberNode 0))

; current_emotion_duration set to default_emotion_duration
; (StateLink (SchemaNode "current emotion duration") (TimeNode 1.0)) ; in seconds
(StateLink (SchemaNode "current emotion duration") (NumberNode 1.0)) ; in seconds

; --------------------------------------------------------
; temp scaffolding and junk.

(define (print-msg node) (display (cog-name node)) (newline) (stv 1 1))
(define (print-atom atom) (format #t "Triggered: ~a \n" atom) (stv 1 1))

; --------------------------------------------------------
; Temporary stand-in for emotion modelling. Right now, just some
; random selectors for classes of emotions.

;; Pick random expression, and display it.
;; line 305 -- pick_random_expression()
(DefineLink
	(DefinedPredicateNode "Pick random positive expression")
	(EvaluationLink (GroundedPredicateNode "py:do_emotion")
		(ListLink
			(RandomChoiceLink
				(ConceptNode "smile")
				(ConceptNode "engaged")
				(ConceptNode "surprised")
			)
			(NumberNode 5.5) ; duration
			(NumberNode 0.7)) ; intensity
	))

(DefineLink
	(DefinedPredicateNode "Pick random negative expression")
	(EvaluationLink (GroundedPredicateNode "py:do_emotion")
		(ListLink
			(RandomChoiceLink
				(ConceptNode "bored")
				(ConceptNode "frustrated")
				(ConceptNode "angry")
			)
			(NumberNode 5.5) ; duration
			(NumberNode 0.7)) ; intensity
	))

;; Pick random gesture
;; XXX really need to pick gestures from a class of appropriate
;; ones, given an emotional state.
;; line 334 -- pick_random_gesture()
(DefineLink
	(DefinedPredicateNode "Pick random gesture")
	(EvaluationLink (GroundedPredicateNode "py:do_gesture")
		(ListLink
			(RandomChoiceLink
				(ConceptNode "nod-1")
				(ConceptNode "nod-2")
				(ConceptNode "yawn-1")
				(ConceptNode "shake-2")
				(ConceptNode "shake-3")
			)
			(NumberNode 0.9) ; duration
			(NumberNode 0.7)))) ; intensity

; ------------------------------------------------------
; TODO --
;
; grep for NumberNode below, and make these (more easily) configurable.
; ------------------------------------------------------

; Return true `fract` percent of the time, else return false.
(define (dice-roll fract)
	(define rrr (random:uniform))
	(cog-logger-info "rando: ~A" rrr)
	(if (> (string->number (cog-name fract)) rrr)
		(stv 1 1) (stv 0 1)))

; line 588 -- dice_roll("glance_new_face")
(DefineLink
	(DefinedPredicateNode "dice-roll: glance")
	(EvaluationLink
		(GroundedPredicateNode "scm: dice-roll")
		(ListLink (NumberNode "0.5"))))

; ------------------------------------------------------
; Basic utilities for working with newly-visible faces.

;; ------
;;
;; Return true if a new face has become visible.
;; line 631, is_someone_arrived
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
	(DefinedPredicateNode "Detected face")
	(SatisfactionLink
		(PresentLink
			(EvaluationLink (PredicateNode "acked face")
					(ListLink (VariableNode "$face-id")))
		)))

;; Randomly select a face out of the crowd.
;; line 747 -- select_a_face_target() and also
;; line 752 -- select_a_glance_target()
(DefineLink
	(DefinedSchemaNode "Select random face")
	(RandomChoiceLink (GetLink
		(EvaluationLink (PredicateNode "acked face")
			(ListLink (VariableNode "$face-id")))
	)))

;; Update the room empty/full status; update the list of acknowledged
;; faces.
;; line 973 clear_new_face_target()
(DefineLink
	(DefinedPredicateNode "Update status")
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "Update room state")
			(TrueLink (PutLink
					(EvaluationLink (PredicateNode "acked face")
							(ListLink (VariableNode "$face-id")))
					(DefinedSchemaNode "New arrivals")))
		)))

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

;; line 742, assign_face_target
;; Send ROS message to look at the face ID.
(DefineLink
	(DefinedSchemaNode "look at person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:look_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New arrivals")
	))

;; line 818, glance_at_new_face
(DefineLink
	(DefinedSchemaNode "glance at person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(DefinedSchemaNode "New arrivals")
	))

; ------------------------------------------------------
; Time-stamp-related stuff.

;; Set a timestamp. XXX todo replace this with timeserver.
;; line 757, timestamp
(DefineLink
	(DefinedSchemaNode "set timestamp")
	(PutLink
		(StateLink (SchemaNode "start-interaction-timestamp")
			(VariableNode "$x"))
		(TimeLink)))

(DefineLink
	(DefinedSchemaNode "get timestamp")
	(GetLink
		(StateLink (SchemaNode "start-interaction-timestamp")
			(VariableNode "$x"))))

;; Evaluate to true, if an expression should be shown.
;; line 933, should_show_expression()
(DefineLink
	(DefinedPredicateNode "Show expression")
	(GreaterThanLink
		(MinusLink
			(TimeLink)
			(DefinedSchemaNode "get timestamp"))
		(GetLink (StateLink (SchemaNode "current emotion duration")
			(VariableNode "$x"))) ; in seconds
	))

; ------------------------------------------------------
; More complex interaction sequences.

;; Interact with the curent face target.
;; line 762, interact_with_face_target()
;; XXX Needs to be replaced by emotional state modelling.
(DefineLink
	(DefinedPredicateNode "Interact with face")
	(SatisfactionLink
		(SequentialAndLink
			;; Look at the interaction face - line 765
			(TrueLink (PutLink
				(EvaluationLink (GroundedPredicateNode "py:look_at_face")
					(ListLink (VariableNode "$face")))
				(GetLink (StateLink interaction-state (VariableNode "$x")))))
			;; line 768
			(DefinedPredicateNode "Pick random positive expression")
			(DefinedPredicateNode "Pick random gesture")
		)))

; ------------------------------------------------------
;; Sequence - if there were no people in the room, then look at the
;; new arrival.
;; line 391 -- owyl.sequence
;; (cog-evaluate! (DefinedPredicateNode "Was Empty Sequence"))
(DefineLink
	(DefinedPredicateNode "Was Empty Sequence")
	(SatisfactionLink
		(SequentialAndLink
			;; line 392
			(DefinedPredicateNode "was room empty?")
			(TrueLink (DefinedSchemaNode "look at person"))
			(TrueLink (DefinedSchemaNode "set timestamp"))
			(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				(ListLink (Node "--- Look at person")))
		)))

;; line 399 -- Sequence - Currently interacting with someone
; (cog-evaluate! (DefinedPredicateNode "Interacting Sequence"))
(DefineLink
	(DefinedPredicateNode "Interacting Sequence")
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "is interacting with someone?")
			(DefinedPredicateNode "dice-roll: glance")
			(TrueLink (DefinedSchemaNode "glance at person"))
			(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				(ListLink (Node "--- Glance at person")))
	)))

;; Respond to a new face becoming visible.
;; line 389 -- Selector
(DefineLink
	(DefinedPredicateNode "Respond to new arrival")
	(SatisfactionLink
		(SequentialOrLink
			(DefinedPredicateNode "Was Empty Sequence")
			(DefinedPredicateNode "Interacting Sequence")
			(EvaluationLink (GroundedPredicateNode "scm: print-msg")
				(ListLink (Node "--- Ignoring new person"))) ; line 406
			(TrueLink)
		)))

;; Check to see if a new face has become visible.
;; line 386 -- someone_arrived()
(DefineLink
	(DefinedPredicateNode "New arrival sequence")
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "Did someone arrive?")
			(DefinedPredicateNode "Respond to new arrival")
			(DefinedPredicateNode "Update status")
		)))

;; Check to see if someone left
;; line 422 -- someone_left()
;; XXX not implemented at all
(DefineLink
	(DefinedPredicateNode "Someone left")
	(FalseLink)
)


;; Start a new interaction
;; line 461 -- sequence ....
; XXX  todo -- check if more than one face target
; record the start time ....
(DefineLink
	(DefinedPredicateNode "Start new interaction")
	(SatisfactionLink
		(SequentialAndLink
			(NotLink (DefinedPredicateNode "is interacting with someone?"))
			(TrueLink (PutLink
				(StateLink interaction-state (VariableNode "$face-id"))
					(DefinedSchemaNode "Select random face")))
		)))

;; Interact with people
;; line 457 -- interact_with_people()
(DefineLink
	(DefinedPredicateNode "Interact with people")
	(SatisfactionLink
		(SequentialAndLink
; XXX incomplete!
			(DefinedPredicateNode "Detected face")
			(TrueLink (DefinedSchemaNode "Select random face"))
			(DefinedPredicateNode "Interact with face")
		)))

;; Nothing is happening
;; line 507 -- nothing_is_happening()
;; XXX Not implemented!
(DefineLink
	(DefinedPredicateNode "Nothing is happening")
	(FalseLink)
)

;; ------------------------------------------------------------------
;; Main loop diagnostics
;; line 988 - idle_spin()
(define loop-count 0)
(define do-run-loop #t)
(define (idle-loop)
	(set! loop-count (+ loop-count 1))
	(format #t "Main loop: ~a\n" loop-count)
	(usleep 1001000)
	(if do-run-loop (stv 1 1) (stv 0 1)))

;; Main loop. Uses tail recursion optimizatio to form the loop.
;; line 556 -- build_tree()
(DefineLink
	(DefinedPredicateNode "main loop")
	(SatisfactionLink
		(SequentialAndLink
			(SequentialOrLink
				(DefinedPredicateNode "New arrival sequence")
				(DefinedPredicateNode "Someone left")
				(DefinedPredicateNode "Interact with people")
				(DefinedPredicateNode "Nothing is happening")
				(TrueLink)
			)
			(EvaluationLink
				(GroundedPredicateNode "scm:idle-loop") (ListLink))
			(EvaluationLink
				(GroundedPredicateNode "py:ros_is_running") (ListLink))
			(DefinedPredicateNode "main loop")
		)))

;; Run the loop (in a new thread)
;; Call (run) to run the loop, (halt) to pause the loop.
;; line 297 -- self.tree.next()
(define (run)
	(set! do-run-loop #t)
	(call-with-new-thread
		(lambda () (cog-evaluate! (DefinedPredicateNode "main loop")))))
(define (halt) (set! do-run-loop #f))
(all-threads)

;
; Silence the output.
(TrueLink)

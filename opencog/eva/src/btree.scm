;
; btree.scm
;
; Experimental behavior tree in the atomspace.
;

(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))
(use-modules (opencog query))
(use-modules (opencog exec))

(load-from-path "utilities.scm")

; (display %load-path)
; (add-to-load-path "../src")
(load-from-path "faces.scm")

; ------------------------------------------------------
; State variables

(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))

;; Assume Eva is sleeping at first
(StateLink soma-state soma-sleeping)

;; Currently, interaction-state will be linked to the face-id of
;; person with whom interaction is taking place.
(define interaction-state (AnchorNode "Interaction State"))
(define no-interaction (ConceptNode "none"))

(StateLink interaction-state no-interaction)

; --------------------------------------------------------
; temp scaffolding and junk.

(define (print-msg) (display "Triggered\n") (stv 1 1))
(define (print-atom atom) (format #t "Triggered: ~a \n" atom) (stv 1 1))

(DefineLink
	(DefinedPredicateNode "Print Msg")
	(EvaluationLink
		(GroundedPredicateNode "scm: print-msg")
		(ListLink))
	)

; ------------------------------------------------------
; TODO --
;
; grep for NumberNode below, and make these (more easily) configurable.
; ------------------------------------------------------

; Return true `fract` percent of the time, else return false.
(define (dice-roll fract)
	(if (> (string->number (cog-name fract)) (random:uniform))
		(stv 1 1) (stv 0 1)))

; line 588 -- dice_roll("glance_new_face")
(DefineLink
	(DefinedPredicateNode "dice-roll: glance")
	(EvaluationLink
		(GroundedPredicateNode "scm: dice-roll")
		(ListLink (NumberNode "0.5"))))

;; ------
;;
;; Return true if interacting with someone.
;; line 650, is_interacting_with_someone
;; (cog-satisfy (DefinedPredicateNode "is interacting with someone?"))
(DefineLink
	(DefinedPredicateNode "is interacting with someone?")
	(NotLink (EqualLink
		(SetLink no-interaction)
		(GetLink (StateLink interaction-state (VariableNode "$x"))))
	))

;;
;; Is the room empty, viz: Does the atomspace contains the link
;; (ListLink (AnchorNode "Room State") (ConceptNode "room empty"))
;; line 665, were_no_people_in_the_scene
(DefineLink
	(DefinedPredicateNode "is room empty?")
	(EqualLink
		(SetLink room-empty)
		(GetLink (StateLink room-state (VariableNode "$x")))
	))

;; line 742, assign_face_target
;; Send ROS message to look at the face ID.
(DefineLink
	(DefinedSchemaNode "look at person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:look_at_face")
			(ListLink (VariableNode "$face")))
		(GetLink
			(EvaluationLink (PredicateNode "visible face")
				(ListLink (VariableNode "$face-id"))))
	))

;; line 818, glance_at_new_face
(DefineLink
	(DefinedSchemaNode "glance at person")
	(PutLink
		(EvaluationLink (GroundedPredicateNode "py:glance_at_face")
			(ListLink (VariableNode "$face")))
		(GetLink
			(EvaluationLink (PredicateNode "visible face")
				(ListLink (VariableNode "$face-id"))))
	))

;; line 757, timestamp
(define (get-timestamp)
	(NumberNode (number->string (current-time))))

(DefineLink
	(DefinedSchemaNode "set timestamp")
	(PutLink
		(EvaluationLink (PredicateNode "start-interaction-timestamp")
			(ListLink (VariableNode "$ts")))
		(ExecutionOutputLink
			(GroundedSchemaNode "scm: get-timestamp")
			(ListLink))))

; ------------------------------------------------------
;; Sequence - if there were no people in the room, then look at the
;; new arrival.
;; line 391 -- owyl.sequence
;; (cog-satisfy empty-seq)
(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			;; line 392
			(DefinedPredicateNode "is room empty?")
			(TrueLink (DefinedSchemaNode "look at person"))
			(TrueLink (DefinedSchemaNode "set timestamp"))
			(DefinedPredicateNode "Print Msg")
		)))

;; line 399 -- Sequence - Currently interacting with someone
; (cog-satisfy interact-seq)
(define interact-seq
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "is interacting with someone?")
			(DefinedPredicateNode "dice-roll: glance")
			(TrueLink (DefinedSchemaNode "glance at person"))
	)))

;
;

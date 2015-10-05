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

; ------------------------------------------------------
; Same as in eva-fsm.scm

; Is the room empty, or is someone in it?
(define room-state (AnchorNode "Room State"))
(define room-empty (ConceptNode "room empty"))
(define room-nonempty (ConceptNode "room nonempty"))

(define soma-state (AnchorNode "Soma State"))
(define soma-sleeping (ConceptNode "Sleeping"))

;; Assume room empty at first
(ListLink room-state room-empty)
(ListLink soma-state soma-sleeping)

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
; TODO -- implement
; (PredicateNode "lookat-face")
; (PredicateNode "glanceat-face")
;
; grep for NumberNode below, and make these (more easily) configurable.
; ------------------------------------------------------

; Return true `fract` percent of the time, else return false.
(define (dice-roll fract)
	(if (> (string->number (cog-name fract)) (random:uniform))
		(stv 1 1) (stv 0 1)))

; line 588 -- dice_roll("glance_new_face")
(DefineLink
	(DefinedSchemaNode "dice-roll: glance")
	(EvaluationLink
		(GroundedPredicateNode "scm: dice-roll")
		(ListLink (NumberNode "0.5"))))

;; ------
;;
;; Is the room empty, viz: Does the atomspace contains the link
;; (ListLink (AnchorNode "Room State") (ConceptNode "room empty"))
;; line 665, were_no_people_in_the_scene
(DefineLink
	(DefinedPredicateNode "is room empty?")
	(EqualLink
		(SetLink room-empty)
		(GetLink (ListLink room-state (VariableNode "$x")))
	))

;; line 742, assign_face_target
;; Set (PredicateNode "lookat-face") to the face ID to look at.
(DefineLink
	(DefinedSchemaNode "look at person")
	(PutLink
		(EvaluationLink (PredicateNode "lookat-face")
			(ListLink (VariableNode "$face")))
		(GetLink
			(EvaluationLink (PredicateNode "visible face")
				(ListLink (VariableNode "$face-id"))))
	))

;; line 818, glance_at_new_face
(DefineLink
	(DefinedSchemaNode "glance at person")
	(PutLink
		(EvaluationLink (PredicateNode "glanceat-face")
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
;; line 392 -- Sequence - if there were no people in the room,
;; then look at the new arrival.
(define empty-seq
	(SatisfactionLink
		(SequentialAndLink
			;; line 392
			(DefinedPredicateNode "is room empty?")
			(DefinedSchemaNode "look at person")
			(DefinedSchemaNode "set timestamp")
			(DefinedPredicateNode "Print Msg")
		)))

;; line 399 -- Sequence - Currently interacting with someone
(define interact-seq
	(SatisfactionLink
		(SequentialAndLink
			(DefinedPredicateNode "is interacting with someone?")
			(DefinedPredicateNode "dice-roll: glance")
			(DefinedPredicateNode "glance at person")
	)))


(cog-satisfy empty-seq)
;
;

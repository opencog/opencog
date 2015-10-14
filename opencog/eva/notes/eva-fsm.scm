;
; Experimental Eva Behavior Finite State Machine (FSM).
;
; A simple finite state machine for controlling Eva's behaviors
;
(add-to-load-path "/usr/local/share/opencog/scm")

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog query))

(load-from-path "utilities.scm")
(load-from-path "faces.scm")

(define eva-trans (ConceptNode "Eva Transition Rule"))

; What emotional state is eva currently displaying?
(define eva-state (AnchorNode "Eva Current State"))
(define eva-bored (ConceptNode "Eva bored"))
(define eva-surprised (ConceptNode "Eva surprised"))

;; Eva's inital state of the FSM
(ListLink eva-state eva-bored)

;; Misc state transistions.
;; if room empty and someone entered -> room non-empty
;; if eva-bored and someone entered -> eva surprised

(define (ola) (display "ola\n"))
(define (bye) (display "bye\n"))

(ContextLink
	(AndLink eva-bored room-nonempty)
	(ListLink eva-trans eva-surprised)
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: ola")
		(ListLink)
	)
)

(ContextLink
	(AndLink eva-surprised room-empty)
	(ListLink eva-trans eva-bored)
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: bye")
		(ListLink)
	)
)

;; Hack job
(define wtf
	(let ((var-eva-state (VariableNode "$eva-state"))
			(var-eva-next-state (VariableNode "$eva-next-state"))
			(var-room-state (VariableNode "$eva-room-state"))
			(var-action (VariableNode "$eva-action")))
		(BindLink
			(VariableList
				var-eva-state var-eva-next-state var-room-state var-action
			)
			(AndLink
				;; If Eva is in the current state ...
				(ListLink eva-state var-eva-state)

				;; ...and the room is in state ...
				(ListLink room-state var-room-state)

				;; ... and there is a transition ...
				(ContextLink
					(AndLink var-eva-state var-room-state)
					(ListLink eva-trans var-eva-next-state)
					var-action
				)
			)
			(AndLink
				;; ... Then, leave the current state ...
				(DeleteLink (ListLink eva-state var-eva-state))

				;; ... And transition to the new state ...
				(ListLink eva-state var-eva-next-state)

				;; and perform the action
				var-action
			)
		)
	)
)


#|
;; Example usage ...

(cog-bind chk-room-empty)
(cog-bind chk-room-non-empty)
(show-room-state)

(cog-incoming-set (PredicateNode "visible face"))


(cog-bind wtf)

(define look-left
	(EvaluationLink
		(GroundedPredicateNode "py: do_look_left")
		(ListLink)))
(define look-right
	(EvaluationLink
		(GroundedPredicateNode "py: do_look_right")
		(ListLink)))
(cog-evaluate! look-left)
(cog-evaluate! look-right)


|#

;; ----

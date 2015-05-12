;
; Exerpimental Eva Behavior Finite State Machine (FSM).
;
; A simple finite state machine for controlling Eva's behaviors
;
(use-modules (opencog))
(use-modules (opencog query))

(define eva-trans (ConceptNode "Eva Transition Rule"))

; What emotional state is eva currently displaying?
(define eva-state (AnchorNode "Eva Current State"))
(define eva-bored (ConceptNode "Eva bored"))
(define eva-surprised (ConceptNode "Eva surprised"))

; Is the room empty, or is someone in it?
(define room-state (AnchorNode "Room State"))
(define room-empty (ConceptNode "room empty"))
(define room-nonempty (ConceptNode "room nonempty"))

;; Eva's inital state of the FSM
(ListLink eva-state eva-bored)

;; Assume room empty at first
(ListLink room-state room-empty)

;; Misc state transistions.
;; if room empty and someone entered -> room non-empty
;; if eva-bored and someone entered -> eva surprised

(ContextLink
	(AndLink eva-bored room-nonempty)
	(ListLink eva-trans eva-surprised)
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: (display \"ola\")")
		(ListLink)
	)
)

(ContextLink
	(AndLink eva-surprised room-empty)
	(ListLink eva-trans eva-bored)
	(ExecutionOutputLink
		(GroundedSchemaNode "scm: (display \"bye\")")
		(ListLink)
	)
)

;; Hack job
(define wtf
	(define var-eva-state (VariableNode "$eva-state"))
	(define var-eva-next-state (VariableNode "$eva-next-state"))
	(define var-room-state (VariableNode "$eva-room-state"))
	(define var-action (VariableNode "$eva-action"))
	(BindLink
		(VariableList
			var-eva-state var-eva-next-state var-room-state
		)
		(ImplicationLink
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

				;; and perform teh action
				var-action
			)
		)
	)
)

; A hacky rule that looks for the visible-face marker, and
; sets the room-is-not-empty flag if a face is visible.
(define chk-room-non-empty
	(BindLink
		(VariableNode "$face-id")
		(ImplicationLink
			(EvaluationLink
				(PredicateNode "visible face")
				(ListLink
					(VariableNode "$face-id")
				)
			)
			; Change the status of the room to "non-empty"
			(AssignLink (TypeNode "ListLink") room-state room-nonempty)
		)
	)
)

; A hack rule that inverts the above.
(define chk-room-empty
	(BindLink
		(VariableNode "$face-id")
		(ImplicationLink
			(AbsentLink
				(EvaluationLink
					(PredicateNode "visible face")
					(ListLink
						(VariableNode "$face-id")
					)
				)
			)
			; Change the status of the room to "empty"
			(AssignLink (TypeNode "ListLink") room-state room-empty)
		)
	)
)

;; Display the current room state
(define (show-room-state)
	(car (cog-chase-link 'ListLink 'ConceptNode room-state)))

;;; Create "my-fsm"
;; (define my-fsm (create-fsm my-trans my-state))


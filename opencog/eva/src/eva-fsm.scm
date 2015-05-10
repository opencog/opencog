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
	(AndLink eva-bored room-non-empty)
	(ListLink eva-trans eva-surprised)
)

(ContextLink
	(AndLink eva-surprised room-empty)
	(ListLink eva-trans eva-bored)
)

;; Hack job
(define wtf
	(define var-eva-state (VariableNode "$eva-state"))
	(define var-eva-next-state (VariableNode "$eva-next-state"))
	(define var-room-state (VariableNode "$eva-room-state"))
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
				)
			)
			(AndLink
				;; ... Then, leave the current state ...
				(DeleteLink (ListLink eva-state var-eva-state))

				;; ... And transition to the new state ...
				(ListLink eva-state var-eva-next-state)
			)
		)
	)
)

; More bad chacking
(define chk-full
	(BindLink
		(VariableNode "$face-id")
	)
	(ImplicationLink
		(EvaluationLink
			(PredicateNode "visible face")
			(ListLink (VariableNode "$face-id"))
		)
		(ListLink room-state room-empty)
	)
)

;;; Create "my-fsm"
;; (define my-fsm (create-fsm my-trans my-state))


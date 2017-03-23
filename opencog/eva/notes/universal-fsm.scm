;
; Universal Finite State Machine (FSM) constructor.
;
; This illustrates the general (universal) FSM state machine constructor.
; This allows mutlple FSM's to be simultaneously defined and operated
; asynchronously from each-other.
;
(use-modules (opencog))
(use-modules (opencog query))

;;; A Universal Deterministic Finite State Machine Constructor.
;;;
;;; This will create a deterministic FSM; that is, a rule that will
;;; transition any arbitrary deterministic FSM from state to state,
;;; given only its name, and the name given to the transition rules.
;;;
;;; Create a BindLink that can take an FSM with the name `fsm-name`
;;; and stores it's state in `fsm-state`.  After the BindLink is
;;; created, each invocation of it will advance the FSM bu one step.
;;;
(define (create-fsm fsm-name fsm-state)
	(BindLink
		;; We will need to find the current and the next state
		(VariableList
			(VariableNode "$curr-state")
			(VariableNode "$next-state")
		)
		(ImplicationLink
			(AndLink
				;; If we are in the current state ...
				(ListLink
					fsm-state
					(VariableNode "$curr-state")
				)
				;; ... and there is a transition to another state...
				(ContextLink
					(VariableNode "$curr-state")
					(ListLink
						fsm-name
						(VariableNode "$next-state")
					)
				)
			)
			(AndLink
				;; ... then transistion to the next state ...
				(ListLink
					fsm-state
					(VariableNode "$next-state")
				)
				;; ... and leave the current state.
				(DeleteLink
					(ListLink
						fsm-state
						(VariableNode "$curr-state")
					)
				)
			)
		)
	)
)

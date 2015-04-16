;
; Very Simple Finite State Machine (FSM) Demo.
;
; This defines a very simple four-state finite state machine: an initial
; state, and a cycle of three state: green, yellow, red. It then uses a
; BindLink to take one step at a time.  The BindLink is more or less
; "universal", in that it can run any FSM, not  just this one. However,
; here it has been a bit simplified, to keep the demo as simple as
; possible. See the file `fsm-full.scm` for the correct way to define a
; truly general-purpose FSM, in such a way that multiple FSM's can be run
; at the same time.
;
(use-modules (opencog))
(use-modules (opencog query))

;; Set of possible states of the state machine
(SetLink
	(ConceptNode "initial state")
	(ConceptNode "green")
	(ConceptNode "yellow")
	(ConceptNode "red")
)

;; The inital state of the FSM
(ListLink
	(AnchorNode "Current State")
	(ConceptNode "initial state")
)

;; The set of allowed state transistions.  Its a triangular cycle,
;; of green goint to yellow going to red going back to green.
;; The intial state transitions into green (and is never visted again).
(ListLink
	(ConceptNode "initial state")
	(ConceptNode "green")
)

(ListLink
	(ConceptNode "green")
	(ConceptNode "yellow")
)

(ListLink
	(ConceptNode "yellow")
	(ConceptNode "red")
)

(ListLink
	(ConceptNode "red")
	(ConceptNode "green")
)


;;; Take one step of a finite state machine.
;;; Note that the algorithm below is "universal": it can run any FSM,
;;; it does not care about the specific states or state transition
;;; rules.
;;;
;;; To define two or more machines that run at the same time, each
;;; should use a different AnchorNode, so that the states of the
;;; various machines would not get confused with one-another.
;;; It would also probably be a good idea to include the machine name
;;; (i.e. the AnchorNode) as part of the transition rules, so that
;;; the transition rules for one machine would not get used accidentally
;;; for another machine.
(define take-one-step
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
					(AnchorNode "Current State")
					(VariableNode "$curr-state")
				)
				;; ... and there is a transition to another state...
				(ListLink
					(VariableNode "$curr-state")
					(VariableNode "$next-state")
				)
			)
			(AndLink
				;; ... then transistion to the next state ...
				(ListLink
					(AnchorNode "Current State")
					(VariableNode "$next-state")
				)
				;; ... and leave the current state.
				(DeleteLink
					(ListLink
						(AnchorNode "Current State")
						(VariableNode "$curr-state")
					)
				)
			)
		)
	)
)

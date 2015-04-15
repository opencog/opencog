;
; Basic unit test of the DeleteLink
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


;;; One step of a finite state machine
;;; Note that the algorithm below is "universal": it can run any FSM,
;;; it does not care about the specific states or state transition
;;; rules.
(define (take-one-step)
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

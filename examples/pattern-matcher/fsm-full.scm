;
; Finite State Machine (FSM) Demo.
;
; Based on fsm-simple.scm, this defines a very simple four-state finite
; state machine, but illustrates the general (universal) FSM state
; transitioner.  This allows mutlple FSM's to be simultaneously defined
; and operated asynchronously from each-other.
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

(define my-fsm (ConceptNode "My FSM"))
(define my-state (AnchoreNode "My FSM's current state"))

;; The inital state of the FSM
(ListLink
	my-state
	(ConceptNode "initial state")
)

;; The set of allowed state transistions.  Its a triangular cycle,
;; of green goint to yellow going to red going back to green.
;; The intial state transitions into green (and is never visted again).
;;
;; Each rule is labelled with the "my-fsm", so that rules for
;; different FSM's do not clash with one-another.  A ConextLink is used
;; because that will allow this example to generalize: Context's are
;; usually used to  express conditional probabilities, so that 
;;
;;     Context  <TV>
;;         A
;;         B
;;
;; representes the probibility of B contiditoned on A, and the TV holds
;; the numeric value for P(B|A).  In this case, A is the current state
;; of the machine, and B the the next state of theh machine, so that P(B|A)
;; is the probability of transitioning to state B give that the machine is
;; in state A.  Such a system is called a Markov Chain.
;; 
;; For the example below, P(B|A) is always one.

(ContextLink
	(ConceptNode "initial state")
	(ListLink
		my-fsm
		(ConceptNode "green")
	)
)

(ContextLink
	(ConceptNode "green")
	(ListLink
		my-fsm
		(ConceptNode "yellow")
	)
)

(ContextLink
	(ConceptNode "yellow")
	(ListLink
		my-fsm
		(ConceptNode "red")
	)
)

(ContextLink
	(ConceptNode "red")
	(ListLink
		my-fsm
		(ConceptNode "green")
	)
)


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

;;; Create "my-fsm"
(define my-fsm (create-fsm my-name my-state))

;;; Take one step.
(bindlink my-fsm)

;;; Take three steps.
;;; Try it!
(bindlink my-fsm)
(bindlink my-fsm)
(bindlink my-fsm)

;
; Probabilistic Finite State Machine (Markov Chain) Demo.
;
; Based on fsm-full.scm, this defines a very simple four-state Markov
; chain, using the same states as the demo FSM's. The difference here is
; that the transitions are specified probabilistically; mutiple
; transitions may occur; each transition has a fixed probability.
;
(use-modules (opencog))
(use-modules (opencog query))

(define my-trans (ConceptNode "My Chain's Transition Rule"))
(define my-state (AnchorNode "My Chain's Current State"))
(define my-nexts (AnchorNode "My Chain's Next State"))

;; The inital state of the FSM.  It starts with 100% probability in this
;; state.
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

; Transition from initial to green with 90% proability.
(ContextLink (stv 0.9 1)
	(ConceptNode "initial state")
	(ListLink
		my-trans
		(ConceptNode "green")
	)
)

; Transition from initial state to yellow with 10% probability.
(ContextLink (stv 0.1 1)
	(ConceptNode "initial state")
	(ListLink
		my-trans
		(ConceptNode "yellow")
	)
)

; Transition from green to yellow with 90% probability
(ContextLink (stv 0.9 1)
	(ConceptNode "green")
	(ListLink
		my-trans
		(ConceptNode "yellow")
	)
)

; Transition from green to red with 10% probability
(ContextLink (stv 0.1 1)
	(ConceptNode "green")
	(ListLink
		my-trans
		(ConceptNode "red")
	)
)

; Transition from yellow to red with 90% probability
(ContextLink (stv 0.9 1)
	(ConceptNode "yellow")
	(ListLink
		my-trans
		(ConceptNode "red")
	)
)

; Transition from yellow to green with 10% probability
(ContextLink (stv 0.1 1)
	(ConceptNode "yellow")
	(ListLink
		my-trans
		(ConceptNode "green")
	)
)

; Transition from red to green with 90% probability
(ContextLink (stv 0.9 1)
	(ConceptNode "red")
	(ListLink
		my-trans
		(ConceptNode "green")
	)
)

; Stay in the red state with 10% probability
(ContextLink (stv 0.1 1)
	(ConceptNode "red")
	(ListLink
		my-trans
		(ConceptNode "red")
	)
)


;;; Create a BindLink that can take an Markov Chain with the name
;;; `fsm-name` and stores it's state in `fsm-state`.  After the
;;; BindLink is created, each invocation of it will advance the
;;; Markov chain one step.
;;;
;;; XXX UNFINISHED --- this requies some new C++ cod before it can work
;;; right. Ask Linsas for the status.
;;;
(define (create-chain-stepper chain-name chain-next chain-state)
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
					chain-state
					(VariableNode "$curr-state")
				)
				;; ... and there is a transition to another state...
				(ContextLink
					(VariableNode "$curr-state")
					(ListLink
						chain-name
						(VariableNode "$next-state")
					)
				)
			)
			;; ... then adjust the probability...
			(ListLink
				chain-next
				(VariableNode "$next-state")
			)
		)
	)
)

;;; Create a BindLink that will find a state vector, and delete it.
;;; After the next chain state is computed, it must be made into the
;;; current chain state.  This is done in a three-step process:
;;; 1) delete the current state vector
;;; 2) copy the next state vector to the current vector
;;; 3) delete the next state-vector.
;;; The below implements steps 1 and 3
(define (create-chain-deleter chain-state)
	(BindLink
		(VariableNode "$state")
		(ImplicationLink
			;; Find the state vector...
			(ListLink
				chain-state
				(VariableNode "$state")
			)
			;; Delete the state vector.
			(DeleteLink
				(ListLink
					chain-state
					(VariableNode "$state")
				)
			)
		)
	)
)

;; Copy a state vector from chain-from to chain-to
(define (create-chain-copier chain-to chain-from)
	(BindLink
		(VariableNode "$state")
		(ImplicationLink
			;; Find the copy-from state vector...
			(ListLink
				chain-from
				(VariableNode "$state")
			)
			;; Copy it to the copy-to state vector.
			(ListLink
				chain-to
				(VariableNode "$state")
			)
		)
	)
)

;; Move a state vector from chain-from to chain-to
;; This combines the copy and delete operation into one.
;; It should be a bit faster.
(define (create-chain-move chain-to chain-from)
	(BindLink
		(VariableNode "$state")
		(ImplicationLink
			;; Find the copy-from state vector...
			(ListLink
				chain-from
				(VariableNode "$state")
			)
			(AndLink
				;; Copy it to the copy-to state vector.
				(ListLink
					chain-to
					(VariableNode "$state")
				)
				;; Delete the copy-from state vector
				(DeleteLink
					(ListLink
						chain-from
						(VariableNode "$state")
					)
				)
			)
		)
	)
)

;;; Create "my-chain"
(define my-stepper (create-chain-stepper my-trans my-nexts my-state))

;;; Take one step.
(cog-prob my-chain)

;;; Take three steps.
;;; Try it!
(cog-prob my-chain)
(cog-prob my-chain)
(cog-prob my-chain)

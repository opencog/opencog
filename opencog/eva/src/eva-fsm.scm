;
; Exerpimental Eva Behavior Finite State Machine (FSM).
;
; A simple finite state machine for controlling Eva's behaviors
;
(use-modules (opencog))
(use-modules (opencog query))

(define my-trans (ConceptNode "My FSM's Transition Rule"))
(define my-state (AnchorNode "My FSM's Current State"))

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
		my-trans
		(ConceptNode "green")
	)
)

(ContextLink
	(ConceptNode "green")
	(ListLink
		my-trans
		(ConceptNode "yellow")
	)
)

(ContextLink
	(ConceptNode "yellow")
	(ListLink
		my-trans
		(ConceptNode "red")
	)
)

(ContextLink
	(ConceptNode "red")
	(ListLink
		my-trans
		(ConceptNode "green")
	)
)


;;; Create "my-fsm"
;; (define my-fsm (create-fsm my-trans my-state))

;;; Take one step.
;(cog-bind my-fsm)

;;; Take three steps.
;;; Try it!
;(cog-bind my-fsm)
;(cog-bind my-fsm)
;(cog-bind my-fsm)

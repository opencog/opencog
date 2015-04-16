;
; Probabilistic Finite State Machine (Markov Chain) Demo.
;
; Based on fsm-full.scm, this defines a very simple four-state Markov
; chain, using the same states as the demo FSM's. The difference here is
; that the transitions are specified probabilistically; mutiple
; transitions may occur; each transition has a fixed probability.
;
; The run this, you probably need to do this:
;
; OCDIR=home/home/yourname/opencog
; export LTDL_LIBRARY_PATH=$OCDIR/build/opencog/guile:$OCDIR/build/opencog/query
;
; Add the following to your ~/.guile file:
; (add-to-load-path "/home/yourname/opencog/build")
; (add-to-load-path "/home/yourname/opencog/opencog/scm")
; (add-to-load-path ".")
;
; Start guile:
; guile
;
; and then load this file:
; (load-from-path "markov-chain.scm")
;
; Then, scroll to the bottom, and some of the commented-out
; examples.

(use-modules (opencog))
(use-modules (opencog query))

(load-from-path "utilities.scm")

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
(define (create-chain-stepper chain-name chain-next chain-state)
	(define curr-state
		(ListLink
			chain-state
			(VariableNode "$curr-state")
		)
	)
	(define state-trans
		(ContextLink
			(VariableNode "$curr-state")
			(ListLink
				chain-name
				(VariableNode "$next-state")
			)
		)
	)
	(define next-state
		(ListLink
			chain-next
			(VariableNode "$next-state")
		)
	)
	(BindLink
		;; We will need to find the current and the next state
		(VariableList
			(VariableNode "$curr-state")
			(VariableNode "$next-state")
		)
		(ImplicationLink
			(AndLink
				;; If we are in the current state ...
				curr-state
				;; ... and there is a transition to another state...
				state-trans
			)
			;; ... then adjust the probability...
			(EvaluationLink
				(GroundedPredicateNode "scm: accum-probability")
				(ListLink
					next-state
					state-trans
					curr-state
				)
			)
		)
	)
)

;;; Get the probability on atom. The probability is assumed to be stroed
;;; in the "mean" component of a SimpleTruthValue.
(define (get-prob atom)
	(assoc-ref (cog-tv->alist (cog-tv atom)) 'mean))

;;; Return true if the TV on the atom is the default TV.
;;; For our purposes, we treat it as the default if the confidence is
;;; below 0.5 (since we later set confidence to 1.0)
(define (is-default-tv? atom)
	(not (< 0.5 (assoc-ref (cog-tv->alist (cog-tv atom)) 'confidence))))

;;; Set a probability value on the atom.
(define (set-prob atom value)
	(cog-set-tv! atom (cog-new-stv value 1.0)))

;;; Accumulate probability (add it to the existing value)
(define (accum-prob atom value)
	(if (is-default-tv? atom)
		(set-prob atom value)
		(set-prob atom (+ (get-prob atom) value))))

;;; Define a function to accumulate the probabilities
;;; It takes as input three atoms: PB, PBA and PA. The truth values on
;;; the last two correspond to P(B|A) (the probability of the state
;;; transition to B, given state A) and P(A) (the probability of being
;;; in state A).  The probability of B is then accumulated:
;;; P(B) += P(B|A) * P(A)
(define (accum-probability PB PBA PA)
	(accum-prob PB (* (get-prob PBA) (get-prob PA)))
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
;(cog-bind my-stepper)

;;; Take three steps.
;;; Try it!
;(cog-bind my-stepper)
;(cog-bind my-stepper)
;(cog-bind my-stepper)

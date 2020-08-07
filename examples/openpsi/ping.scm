(use-modules (opencog)
             (opencog openpsi))

; Ball states
(define neutral (Concept "neutral"))
(define pinged (Concept "pinged"))

; Define the initial state of the ball.
(define ball (Concept "ball"))
(State ball neutral)

(define state-var (Variable "state"))

; ----------------------------------------------------------------------
; DEFINING PINGING
; ----------------------------------------------------------------------
; Define ping contexts
(define ping-context-1 (list
  (State ball state-var)
  (Equal state-var neutral)))

; Define ping action
(define (ping)
  (sleep 5)
  (display "\nJust pinged\n")
  (State ball pinged))

(define ping-action
  (ExecutionOutput
    (GroundedSchema "scm: ping")
    (List)))

; Define ping goal
(define ping-goal (psi-goal "ping" 0))

; Define ping component that uses the default step `psi-step` and default
; action-selector `psi-get-satisfiable-rules`.
(define ping-component (psi-component "ping"))

; Define ping rules
; The truthvalue for the rule can be used by the action selector for determing
; which action to choose. If you choose to use it when defining your
; action-selector.
(psi-rule ping-context-1 ping-action ping-goal (stv 1 1) ping-component)

; Start ping components
; (psi-run ping-component)

; Stop ping components
; (psi-halt ping-component)

(use-modules (opencog)
             (opencog openpsi))

;(use-modules (opencog logger))
;(cog-logger-set-level! (psi-get-logger) "debug")
;(cog-logger-set-stdout! (psi-get-logger) #t)

; Ball states
(define neutral (Node "neutral"))
(define pinged (Node "pinged"))
(define ponged (Node "ponged"))

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

(define ping-context-2 (list
  (State ball state-var)
  (Equal state-var ponged)))

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
(define ping-goal (Concept "ping"))

; Define ping rules
(define ping-component (psi-component  "ping"))

; The truthvalue for the rule can be used by the action selector for determing
; which action to choose. If you choose to use it when defining your
; action-selector.
(psi-rule ping-context-1 ping-action ping-goal (stv 1 1) ping-component)
(psi-rule ping-context-2 ping-action ping-goal (stv 1 1) ping-component)

; Define ping's action-selector
(define (ping-action-selector)
  (psi-get-satisfiable-rules ping-component))

(psi-set-action-selector!
  ping-component
  (ExecutionOutput (GroundedSchema "scm: ping-action-selector") (List)))

; ----------------------------------------------------------------------
; DEFINING PONGING
; ----------------------------------------------------------------------
; Define pong contexts
(define pong-context (list
  (State ball state-var)
  (Equal state-var pinged)))

; Define ping action
(define (pong)
  (sleep 5)
  (display "\nJust ponged\n")
  (State ball ponged))

(define pong-action
  (ExecutionOutput
    (GroundedSchema "scm: pong")
    (List)))

; Define pong goal
(define pong-goal (Concept "pong"))

; Define pong rules
(define pong-component (psi-component  "pong"))

(psi-rule pong-context pong-action pong-goal (stv 1 1) pong-component)

; Define pong's action-selector
(define (pong-action-selector)
  (psi-get-satisfiable-rules pong-component))

(psi-set-action-selector!
  pong-component
  (ExecutionOutput (GroundedSchema "scm: pong-action-selector") (List)))

; ----------------------------------------------------------------------
; Start ping pong components
; ----------------------------------------------------------------------
(psi-run pong-component)
(psi-run ping-component)

; For stopping the components use psi-halt
; (psi-halt ping-component)
; (psi-halt pong-component)

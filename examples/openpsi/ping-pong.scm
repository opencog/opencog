(use-modules (opencog)
             (opencog openpsi))

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
(define ping-goal (psi-goal "ping" 0))

; Define ping-component that uses the default step `psi-step` and default
; action-selector `psi-get-satisfiable-rules`.
(define ping-component (psi-component "ping"))

; Define ping rules
; The truthvalue for the rule can be used by the action selector for determing
; which action to choose. If you choose to use it when defining your
; action-selector.
(psi-rule ping-context-1 ping-action ping-goal (stv 1 1) ping-component)
(psi-rule ping-context-2 ping-action ping-goal (stv 1 1) ping-component)

; ----------------------------------------------------------------------
; DEFINING PONGING
; ----------------------------------------------------------------------
; Define pong contexts
(define pong-context (list
  (State ball state-var)
  (Equal state-var pinged)))

; Define pong goal
(define pong-goal (psi-goal "pong" 1))

; Define ping action
(define (pong)
  (sleep 1)
  (display "\nJust ponged\n")
  (State ball ponged)
  ; The side-effect of the action decreases the urge.
  (psi-decrease-urge pong-goal 1))

(define pong-action
  (ExecutionOutput
    (GroundedSchema "scm: pong")
    (List)))

; Define pong-component that uses custom step in place of `psi-step` and default
; action-selector `psi-get-satisfiable-rules`.
(define (pong-step)
  (sleep 3)
  (let ((urge (psi-urge pong-goal)))
    (if (< urge 0.7)
      (begin
        (format #t "\nNot yet feeling like ponging the ball. Urge = ~a\n" urge)
        (psi-increase-urge pong-goal 0.2))
      (begin
        (format #t "\nFeeling like ponging the ball. Urge = ~a\n" urge)
        (psi-increase-urge pong-goal 0.2)
        (psi-step (Concept "pong"))))
  )
  (stv 1 1))


(define pong-steper
  (Evaluation
    (GroundedPredicate "scm: pong-step")
    (List)))

(define pong-component (psi-component  "pong" pong-steper))

; Replace the default action-selector for the pong-component.
(define (pong-action-selector)
  (psi-get-satisfiable-rules pong-component))

(psi-set-action-selector!
  pong-component
  (ExecutionOutput (GroundedSchema "scm: pong-action-selector") (List)))

; Define pong rules
(psi-rule pong-context pong-action pong-goal (stv 1 1) pong-component)

; ----------------------------------------------------------------------
; Start ping and pong components
(psi-run pong-component)
(psi-run ping-component)

; Stop ping and pong components
; (psi-halt ping-component)
; (psi-halt pong-component)

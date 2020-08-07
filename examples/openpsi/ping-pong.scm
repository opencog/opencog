(use-modules (opencog)
             (opencog openpsi))

; This expands on the `ping.scm` example by adding a pong component. Thus,
; start by loading it.
(load "ping.scm")

; Exapnd ball states
(define ponged (Concept "ponged"))

; Expand pinging rules
(define ping-context-2 (list
  (State ball state-var)
  (Equal state-var ponged)))

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

; Define pong component that uses custom step in place of `psi-step` and
; default action-selector `psi-get-satisfiable-rules`.
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


(define pong-stepper
  (Evaluation
    (GroundedPredicate "scm: pong-step")
    (List)))

(define pong-component (psi-component "pong" pong-stepper))

; Replace the default action-selector of the pong component.
(define (pong-action-selector)
  (psi-get-satisfiable-rules pong-component))

(psi-set-action-selector!
  pong-component
  (ExecutionOutput (GroundedSchema "scm: pong-action-selector") (List)))

; Define pong rules
(psi-rule pong-context pong-action pong-goal (stv 1 1) pong-component)

; ----------------------------------------------------------------------
; Start ping and pong components
; (psi-run pong-component)
; (psi-run ping-component)

; Stop ping and pong components
; (psi-halt ping-component)
; (psi-halt pong-component)

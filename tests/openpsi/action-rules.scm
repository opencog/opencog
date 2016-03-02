; Copyright (C) 2016 OpenCog Foundation

; Define an OpenPsi-Demand called Energy

; Specify what kind of default behavior the demand should have, using one of
; the 3 options provided. You can define your own behavior.
(define energy-default-action  (psi-action-minimize 5))

; Define the Energy demand
(define energy (psi-demand "Energy" .71 energy-default-action))

; Add action for increasing the energy-demand.
(psi-action-rule-maximize energy 10)
(psi-action-rule-maximize energy 5)

; Add action for decreasing the energy-demand.
(psi-action-rule-minimize energy 6)
(psi-action-rule-minimize energy 5)

; --------------------------------------------------------------
; For test_psi_get_action_rules
(define (test-def) (psi-get-action-rules-typed energy "Default"))
(define (test-def-result)
    (SetLink (DefinedSchemaNode "OpenPsi: Energy-action-rule-Default"))
)

(define (test-inc) (psi-get-action-rules-typed energy "Increase"))
(define (test-inc-result)
    (SetLink
      (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-10")
      (DefinedSchemaNode "OpenPsi: Energy-action-rule-maximize-5")
    )
)

(define (test-dec) (psi-get-action-rules-typed energy "Decrease"))
(define (test-dec-result)
    (SetLink
      (DefinedSchemaNode "OpenPsi: Energy-action-rule-minimize-6")
      (DefinedSchemaNode "OpenPsi: Energy-action-rule-minimize-5")
    )
)

; --------------------------------------------------------------
; For test_psi_get_demands
(define (test-select-all)
    ; Using psi-get-demands b/c it is just a wrapper.
    (psi-get-demands-all)
)

(define (test-select-all-result)
    (SetLink
      (ConceptNode "OpenPsi: Energy" (stv 0.710000 1.000000))
    )
)

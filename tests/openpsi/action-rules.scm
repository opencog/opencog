;Copyright (C) 2016 OpenCog Foundation

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

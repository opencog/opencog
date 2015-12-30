; Copyright (C) 2015 OpenCog Foundation

(load-from-path "openpsi/active-schema-pool.scm")
(load-from-path "openpsi/demand.scm")





; Define an OpenPsi-Demand Energy.
(define energy (psi-demand "Energy" .71 (psi-action-keep-within .5 .7 5)))

; Add actions that affect Energy.
#!


(define (psi-demand-minimize rate)
"
  Returns an action for This minimizes the it is associated with
"

)
; Define the OpenPsi-Demand Affiliation
(psi-demand "Affiliation" .6 .8 1)

; Define the OpenPsi-Demand Certainty
(psi-demand "Certainty" .6 .8 1)

; Define the OpenPsi-Demand Competence
(psi-demand "Competence" .6 .8 1)


; Define the OpenPsi-Demand Integrity
(psi-demand "Integrity" .6 .8 1)
!#

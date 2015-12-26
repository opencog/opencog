(use-modules (opencog rule-engine))

(load-from-path "openpsi/demand/updater-rule.scm")
(load-from-path "openpsi/modulator/updater-rule.scm")

(define psi-active-schema-pool
    (ConceptNode (string-append (psi-prefix-str) "active-schema-pool")))

; Define OpenPsi active-schema-pool as the ure rulebase. The iternation
; has no effect as all rules are run.
(define openpsi-rbs (ure-define-rbs psi-active-schema-pool 1))

; Add default rules
(ure-add-rule openpsi-rbs "psi-demand-updater-rule"
    psi-demand-updater-rule 1)

(ure-add-rule openpsi-rbs "psi-modulator-updater-rule"
    psi-modulator-updater-rule 1)

#!
; the intention is to use atomese to define the function to be used for
; goal and action selection.
(DefineLink
    (ConceptNode
        (string-append (psi-prefix-str) "default-asp-goal-selector-gpn"))
    (GroundedPredicateNode "scm: psi-lowest-demand?"))
!#

; Copyright (C) 2015 OpenCog Foundation

(use-modules (opencog rule-engine))

(load-from-path "openpsi/active-schema-pool.scm")
(load-from-path "openpsi/demand.scm")

(define (psi-demand-keep-within min-value max-value)
    (ExecutionOutputLink
        (GroundedSchemaNode "scm: psi-demand-updater") ; pre-condition
        (ListLink
            (VariableNode "Demand")
            (VariableNode "min_acceptable_value")
            (VariableNode "max_acceptable_value")))
)

(define (psi-demand-updater demand min-acceptable-value max-acceptable-value)
    (let ((current-value (tv-mean (cog-tv  demand)))
          (min-value (string->number (cog-name min-acceptable-value)))
          (max-value (string->number (cog-name max-acceptable-value)))
         )
         ; TODO:
         ;1. check the 'fuzzy_within' equations, it seems too have small effect.
         ;2. make it easier for different demands have differing updaters, as
         ;   different characters controled are likely have different
         ;   personality.
         (cog-set-tv! demand
             (stv (fuzzy_within current-value min-value max-value 100) 1)
         )
     )
     #t
)


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

; --------------------------------------------------------------

; Define the OpenPsi-Demand Affiliation
(define-psi-demand "Affiliation" .6 .8 1)

; Define the OpenPsi-Demand Certainty
(define-psi-demand "Certainty" .6 .8 1)

; Define the OpenPsi-Demand Competence
(define-psi-demand "Competence" .6 .8 1)

; Define the OpenPsi-Demand Energy
(define-psi-demand "Energy" .71 .5 .7)

; Define the OpenPsi-Demand Integrity
(define-psi-demand "Integrity" .6 .8 1)

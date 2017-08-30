; Copyright (C) 2016 OpenCog Foundation

; --------------------------------------------------------------
(define-module (opencog openpsi)
  #:use-module (opencog)
  #:export (
    ; From action-selector.scm
    psi-set-action-selector psi-get-action-selector
    psi-select-rules-per-demand

    ; From demand.scm
    psi-get-all-demands psi-get-all-enabled-demands psi-get-all-valid-demands
    psi-demand psi-demand? psi-set-demand-value psi-goal-increase
    psi-demand-value-increase psi-goal-decrease psi-demand-value-decrease
    psi-demand-skip psi-demand-skip?

    ; From control.scm
    psi-set-updater! psi-get-updater psi-controller psi-controller-idle
    psi-controller-busy psi-controller-occupy psi-controller-release
    psi-rule-set-atomese-weight psi-set-controlled-rule
    psi-get-controlled-rules psi-rule-atomese-weight
    psi-controller-update-weights psi-rule-disable psi-rule-enable

    ; From rule.scm
    psi-get-rules psi-get-all-rules psi-rule? psi-get-all-actions psi-action?
    psi-get-context psi-get-action psi-get-goal psi-rule-alias
    psi-partition-rule-with-alias psi-related-goals psi-satisfiable?
    psi-rule-satisfiability psi-get-satisfiable-rules
    psi-get-weighted-satisfiable-rules psi-get-all-satisfiable-rules
    psi-get-all-weighted-satisfiable-rules psi-context-weight psi-action-weight

    ; From main.scm
    psi-running? psi-get-loop-count psi-run-continue? psi-step psi-run psi-halt
    psi-get-logger

    ; From utilities.scm
    psi-prefix-str psi-suffix-str psi-get-exact-match psi-get-dual-match
    psi-get-members
    )
)

(load-from-path "opencog/openpsi/action-selector.scm")
(load-from-path "opencog/openpsi/demand.scm")
(load-from-path "opencog/openpsi/control.scm")
(load-from-path "opencog/openpsi/rule.scm")
(load-from-path "opencog/openpsi/main.scm")
(load-from-path "opencog/openpsi/utilities.scm")

; Copyright (C) 2016 OpenCog Foundation

; --------------------------------------------------------------
(define-module (opencog openpsi))

(use-modules (opencog))

(load-from-path "opencog/openpsi/action-selector.scm")
(load-from-path "opencog/openpsi/demand.scm")
(load-from-path "opencog/openpsi/control.scm")
(load-from-path "opencog/openpsi/rule.scm")
(load-from-path "opencog/openpsi/main.scm")
(load-from-path "opencog/openpsi/utilities.scm")

(define-public (load-openpsi-in-development)

; This is a temporary hack to get past installing openpsi to get access to
; atom-types module for PredictiveImplicationLink
    (load-from-path "opencog/openpsi/modulator.scm")
    (load-from-path "opencog/openpsi/sec.scm")
    (load-from-path "opencog/openpsi/interaction-rule.scm")
    (load-from-path "opencog/openpsi/updater.scm")
)

(export)

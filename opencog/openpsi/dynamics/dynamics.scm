; Copyright (C) 2017 OpenCog Foundation

(define-module (opencog openpsi dynamics)
  #:use-module (srfi srfi-1)
  #:use-module (opencog)
  #:use-module (opencog openpsi)
  #:export (
    ; From emotion.scm
    psi-create-emotion psi-get-emotions

    ; From event.scm
    psi-create-monitored-event psi-get-monitored-events
    psi-most-recent-occurrence-pred psi-set-event-occurrence!

    ; From interaction-rule.scm
    psi-create-general-rule psi-create-interaction-rule

    ; From modulator.scm
    psi-create-modulator psi-get-modulators psi-get-arousal psi-get-pos-valence
    psi-get-neg-valence psi-get-resolution-level psi-get-selection-threshold
    psi-get-securing-threshold psi-default-modulator-alist

    ; From sec.scm
    psi-create-sec psi-create-stimulus-sec psi-get-secs psi-is-sec?
    psi-default-sec-alist

    ; From updater.scm
    psi-set-event-callback! do-psi-updater-step adjust-psi-var-level
    psi-ultradian-update psi-noise-update get-random-cycle-offset
    psi-updater-running? psi-updater-run psi-updater-halt
    psi-set-expression-callback! psi-get-interaction-rules

    ; From utilities.scm
    psi-get-number-values-for-vars psi-baseline-value-node
    psi-set-baseline-value! psi-get-baseline-value psi-get-value
    psi-set-value! psi-get-number-value
    )
)

(load-from-path "opencog/openpsi/dynamics/emotion.scm")
(load-from-path "opencog/openpsi/dynamics/event.scm")
(load-from-path "opencog/openpsi/dynamics/interaction-rule.scm")
(load-from-path "opencog/openpsi/dynamics/modulator.scm")
(load-from-path "opencog/openpsi/dynamics/sec.scm")
(load-from-path "opencog/openpsi/dynamics/updater.scm")
(load-from-path "opencog/openpsi/dynamics/utilities.scm")

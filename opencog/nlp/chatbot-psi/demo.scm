(use-modules (ice-9 receive))
(use-modules (opencog) (opencog openpsi))

; --------------------------------------------------------------
; NOTE: Disable the high-level loop that integrates multiple components before
; disabling sub-component, i.e. (psi-halt) first. When starting do the opposite.
; This hasn't been tested.

; --------------------------------------------------------------
(define-public (enable-pln-demo-only)
"
  Enables the openpsi-pln rules and the openpsi-aiml rules only. The aiml rules
  are enalbed b/c they are the primary chat interface.
"
    ; Stop the loops.
    (psi-halt)
    (pln-halt)

    ; Make the weight changes needed for configuration.
    (disable-all-demos)
    (psi-rule-enable "select_pln_answer" (psi-get-controlled-rules))
    (psi-rule-enable "aiml" (psi-get-controlled-rules))

    ; Start the loops.
    (pln-run)
    (psi-run)
)

; --------------------------------------------------------------
(define-public (enable-all-demos)
"
  This is the default mode. All the rules are given a weight of 0.9.
"
    ; Stop the loops.
    (psi-halt)
    (pln-halt)

    ; Make the weight changes needed for configuration.
    (disable-all-demos)
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            other)
    )

    ; Start the loops.
    (pln-run)
    (psi-run)
)

; --------------------------------------------------------------
(define-public (disable-all-demos)
"
  This is run when disabling all the rules, when switching between modes.
  When disabling the rules, their weight is set to zero.

  When adding new demo modes, make sure you run (psi-halt) after calling
  this function.
"
    ; Stop the loops.
    (psi-halt)
    (pln-halt)

    ; Make the weight changes needed for configuration.
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            other)
    )

    ; OpenPsi is restarted because there might be rules that are not controlled
    ; that should keep on running.
    (psi-run)
)

; --------------------------------------------------------------

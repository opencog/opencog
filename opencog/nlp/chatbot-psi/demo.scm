(use-modules (ice-9 receive))
(use-modules (opencog) (opencog openpsi))

;TODO: control this using chatbot-eva.

(define-public (enable-pln-demo-only)
"
  Enables the openpsi-pln rules and the openpsi-aiml rules only.
"
    (psi-halt)
    (receive (filtered other)
        (psi-partition-rule-with-alias "select_pln_answer"
            (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            filtered)
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            other)
    )

    (psi-rule-enable "aiml" (psi-get-controlled-rules))
    (pln-run)
    (psi-run)
)


(define-public (enable-all-demos)
    (psi-halt)
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            other)
    )

    (pln-run)
    (psi-run)
)

(define-public (disable-all-demos)
    (psi-halt)
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            other)
    )

    (pln-run)
    (psi-run)
)

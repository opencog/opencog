(use-modules (srfi srfi-1))
(use-modules (ice-9 receive))

(use-modules (opencog))
(use-modules (opencog exec))
(use-modules (opencog rule-engine))

; ----------------------------------------------------------------------------
(define-public (pln-load-rules RULE-TYPE)
"
  pln-load-rules RULE-TYPE

  Loads the different variations of the rules known by RULE-TYPE. A RULE-TYPE
  may include the categorization of the rule. For example, 'term/deduction'
  implies that the rule to be loaded is the term-logic deduction rule.
"
  ; NOTE:
  ; 1. If a rule needs formula defined in formulas.scm then the rule file
  ;    should load it.
  ; 2. Rule files are assumed to be named as "RULE-TYPE.scm"
  ; 3. load-from-path is used so as to be able to use build_dir/opencog/scm,
  ;    even when the module isn't installed.

  (load-from-path (string-append "opencog/pln/rules/" RULE-TYPE ".scm"))
)

; ----------------------------------------------------------------------------
(define (simple-forward-step RB-NODE FOCUS-SET)
"
  Makes a single forward step using the rules in rulebase RB-NODE over the
  whole FOCUS-SET.
"
; NOTE: It is simple b/c it doesn't try to restrict inference over a
; certain source atoms.
; TODO: Move logic to ForwardChainer.
    (let* ((result (cog-fc RB-NODE (Set) #:focus-set (Set FOCUS-SET)))
           (result-list (cog-outgoing-set result)))
        ; Cleanup
        (cog-delete result)

        ; If there are multiple results for application of a rule, the
        ; result will have a ListLink of the results. Get the results out
        ; of the ListLinks helps in debugging and filtering-for-pln/sureal
        (receive (list-links other)
            (partition
                (lambda (x) (equal? 'ListLink (cog-type x))) result-list)

            (let ((partial-results (append-map cog-outgoing-set list-links)))
                ; Cleanup. NOTE: Cleanup is not done on `other` b/c, it might
                ; contain atoms which are part of r2l outputs, which if deleted
                ; recursively might affect the nlp pipline.
                (map cog-delete-recursive list-links)
                (delete-duplicates (append partial-results other))
            )
        )
    )
)

; ----------------------------------------------------------------------------
(define-public (simple-forward-chain RB-NODE FOCUS-SET STEPS)
"
  Applys the rules in rulebase RB-NODE over the whole FOCUS-SET, STEPS times.
  Before each recursive step occurs, the FOCUS-SET and outputs of current-step
  are merged and passed as the new FOCUS-SET.

  Returns a list containing both the FOCUS-SET and the inference results.
"
    ; TODO: Add an optional argument for filtering results b/n steps using.
    ; Create the next focus-set.
    (define (create-next-fs prev-fs chaining-result)
            (delete-duplicates (append chaining-result prev-fs)))

    (if (equal? 1 STEPS)
        (create-next-fs  FOCUS-SET (simple-forward-step RB-NODE FOCUS-SET))
        (simple-forward-chain RB-NODE
            (create-next-fs FOCUS-SET (simple-forward-step RB-NODE FOCUS-SET))
            (- STEPS 1))
    )
)

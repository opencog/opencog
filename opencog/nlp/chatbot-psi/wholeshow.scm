(use-modules (ice-9 receive))
(use-modules (opencog) (opencog openpsi))

(load "pln-reasoner.scm")

; --------------------------------------------------------------
; NOTE: Disable the high-level loop that integrates multiple components before
; disabling sub-component, i.e. (psi-halt) first. When starting do the opposite.
; This hasn't been tested.

; --------------------------------------------------------------
; Node used to communicate what the type of wholeshow is.
(define wholeshow-state (Anchor "wholeshow-state"))

; Node used to communicate what the previous wholeshow type was.
(define previous-wholeshow-state (Anchor "previous-wholeshow-state"))

; Set default state
(State wholeshow-state (Node "default"))
(State previous-wholeshow-state (Node "default"))

; --------------------------------------------------------------
; Define helper functions for updating weight of controlled-rules
; --------------------------------------------------------------
(define-public (wholeshow-modes)
"
  Returns a a-list of lowercase names of the avilable modes for keys and
  procedures for values.
"
    (list
        (cons "reasoning" enable-pln-demo)
        (cons "philosophy" enable-philosophy-demo)
        (cons "default" enable-all-demos))
)

; --------------------------------------------------------------
(define-public (enable-all-demos)
"
  This is the default mode. All the rules are given a weight of 0.9.
"
    ; Make the weight changes needed for configuration.
    (disable-all-demos)
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            other)
    )
)

; --------------------------------------------------------------
(define-public (disable-all-demos)
"
  This is run when disabling all the rules, when switching between modes.
  When disabling the rules, their weight is set to zero.

  When adding new demo modes, make sure you run (psi-halt) after calling
  this function.
"
    ; Make the weight changes needed for configuration.
    (receive (filtered other)
        (psi-partition-rule-with-alias "" (psi-get-controlled-rules))
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            other)
    )
)

; --------------------------------------------------------------
(define-public (show-demo-state)
"
  Returns an a-list with rule aliases for keys and their weights for values.
"
    (define result '())
    (let ((rules (psi-get-controlled-rules)))
        (for-each (lambda (x) (set! result
            (assoc-set! result
                (psi-suffix-str (cog-name  (car (psi-rule-alias x))))
                (cog-stv-strength x))))
            rules)
        result
    )
)

; --------------------------------------------------------------
(define-public (enable-pln-demo)
"
  Enables the openpsi-pln rules and the openpsi-aiml rules only. The aiml rules
  are enalbed b/c they are the primary chat interface.
"
    ; Make the weight changes needed for configuration.
    (disable-all-demos)
    (psi-rule-enable "select_pln_answer" (psi-get-controlled-rules))
    (psi-rule-enable "aiml" (psi-get-controlled-rules))
)

; --------------------------------------------------------------
(define-public (enable-philosophy-demo)
"
  Enables the random_sentence_pkd and random_sentence_blogs rules.
"
    ; Make the weight changes needed for configuration.
    (disable-all-demos)
    (psi-rule-enable "random_sentence_pkd" (psi-get-controlled-rules))
    (psi-rule-enable "random_sentence_blogs" (psi-get-controlled-rules))
)

; --------------------------------------------------------------
(define-public (switch-wholeshow-mode)
    (let ((ws-mode (cog-name (car
            (cog-chase-link 'StateLink 'Node wholeshow-state)))))

        ((assoc-ref (wholeshow-modes) ws-mode))
        (State previous-wholeshow-state (Node ws-mode))
        (stv 1 1)
    )
)

; --------------------------------------------------------------
; Define helper functions for context checking
; --------------------------------------------------------------
(define (enable-pattern)
"
  The command pattern used for enabling a demo.
"
    (List
        (Word "let")
        (Word "us")
        (Word "show")
        (Glob "wholeshow-mode"))
)

(define (disable-pattern)
"
  The command pattern used for switching to default demo mode.
"
    (List
        (Word "we")
        (Word "are")
        (Word "done")
        (Glob "wholeshow-mode"))
)

(define (get-utterance-pattern pattern-list)
    (cog-execute! (Get
        (State input-utterance-words pattern-list)))
)

(define (get-words list-link)
    (map
        (lambda (x) (string-downcase (cog-name x)))
        (cog-outgoing-set list-link))
)

(define (has-word? word list) (if (member word list) #t #f))

(define (get-chosen-mode set-link)
    (let ((possible-mode (get-words (gar set-link))))
        (filter (lambda (x) (member x possible-mode))
            (map car (wholeshow-modes)))
    )
)

(define (is-wholeshow-action-possible? set-link)
    (if  (and (not (null? (gar set-link)))
            (not (null? (get-chosen-mode set-link))))
        #t
        #f
    )
)

(define-public (utterance-matches-wholeshow-pattern?)
    (let ((d-result (get-utterance-pattern (disable-pattern)))
        (e-result (get-utterance-pattern (enable-pattern))))

        (cond
            ((and (not (equal? (Set) d-result))
                (is-wholeshow-action-possible? d-result))
                    (State wholeshow-state (Node "default"))
                    (stv 1 1)
                )
            ((and (not (equal? (Set) e-result))
                (is-wholeshow-action-possible? e-result))
                    (State
                        wholeshow-state
                        (Node (car (get-chosen-mode e-result))))
                    (stv 1 1)
                )
            (else (stv 0 1))
        )
    )
)

; --------------------------------------------------------------
; Used for checking if the input utterance is a wholeshow command
(Define
    (DefinedPredicate "utterance-matches-wholeshow-pattern?")
    (Evaluation
        (GroundedPredicate "scm: utterance-matches-wholeshow-pattern?")
        (List))
)

(Define
    (DefinedPredicate "wholeshow-change-requested?")
    (Not (Equal
        (Get
            (TypedVariable (Variable "current-state") (Type "Node"))
            (State wholeshow-state (Variable "current-state")))
        (Get
            (TypedVariable (Variable "previous-state") (Type "Node"))
            (State previous-wholeshow-state (Variable "previous-state")))
    ))
)

(Define
    (DefinedPredicate "wholeshow-action")
    (Evaluation
        (GroundedPredicate "scm: switch-wholeshow-mode")
        (List))
)

(Define
    (DefinedPredicate "wholeshow-updater")
    (SequentialAnd
        (DefinedPredicate "utterance-matches-wholeshow-pattern?")
        (DefinedPredicate "wholeshow-change-requested?")
        (DefinedPredicate "wholeshow-action"))
)

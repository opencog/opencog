; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog exec))

(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define (psi-action-rule-selector-pattern)
"
  This returns the StateLink that is used for specifying the action selecting
  evaluatable term.

  A StateLink is used instead of an InheritanceLink because there could only
  be one active action-rule-selector at a time eventhough there could be
  multiple possible action-rule-selectors. And this enables dynamically
  changing the action-rule-selector through learning.
"
    (StateLink
        (ConceptNode (string-append (psi-prefix-str) "action-rule-selector"))
        (VariableNode "$dpn")
    )
)

; --------------------------------------------------------------
(define (psi-action-rule-selector-set! dpn)
"
  Sets the given DefinedPredicateNode to be used for selecting action-rule.

  dpn:
  - The DefinedPredicateNode that represents the evaluatable-term used for
    selecting the action-rules that should be part of the asp.
"
    ; Check arguments
    (if (not (equal? (cog-type dpn) 'DefinedPredicateNode))
        (error "Expected DefinedPredicateNode got: " dpn))

    (cog-execute!
        (PutLink
            (psi-action-rule-selector-pattern)
            dpn)
    )
)


; --------------------------------------------------------------
(define (psi-add-action-rule-selector eval-term name)
"
  Returns the DefinedPredicateNode that represents the evaluatable term
  after defining it as an opencog goal-selector.

  eval-term:
  - An evaluatable term.

  name:
  -  A string for naming the action-rule-selector. The name will be prefixed
     by the following string `OpenPsi: action-rule-selector-`.
"
    ; Check arguments
    (if (not (string? name))
        (error "Expected second argument to be a string, got: " name))

    ; TODO: Add checks to ensure the eval-term argument is actually evaluatable
    (let* ((z-name (string-append
                        (psi-prefix-str) " action-rule-selector-" name))
           (selector-dpn (cog-node 'DefinedPredicateNode z-name)))
       (if (null? selector-dpn)
           (begin
               (set! selector-dpn (DefinedPredicateNode z-name))
               (DefineLink selector-dpn eval-term)

                (EvaluationLink
                    (PredicateNode "rule-selector-for")
                    (ListLink selector-dpn (psi-asp)))

                selector-dpn
           )
           ; NOTE: Assuming that it is highly unlikely that the same node
           ;  wouldn't be used for another purpose.
           selector-dpn
       )
    )
)

; --------------------------------------------------------------
(define (psi-select-action-rules)
"
  Selects all actions of current effect type and update the psi-asp.
"
    (define (get-as) ; get the action-rule-selctor
        (cog-outgoing-set (cog-execute!
            (GetLink (psi-action-rule-selector-pattern)))))

    (let ((goal (psi-current-goal))
          (effect-type (psi-current-effect-type))
          (action-selector (get-as))
          (asp (psi-asp)))
        (cond
            ((null? action-selector)
                (error "In procedure psi-select-action-rules: "
                       "action-selector is not set."))
            ((equal? effect-type "Default")
            ; If default effect-type then add only the default actions.
                (psi-update-asp  asp (psi-get-action-rules-default)))
            (else (psi-update-asp asp (cog-outgoing-set
                    (psi-get-action-rules (car action-selector) goal))))
        )
    )
)

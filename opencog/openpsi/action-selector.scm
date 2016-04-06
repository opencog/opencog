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
(define-public (psi-action-rules-typed-term demand-node effect-type)
"
  Returns evaluatable-term used to choose certain typed action-rules.
"
    (PresentLink
        (EvaluationLink
            (PredicateNode (string-append (psi-prefix-str) effect-type))
            (ListLink (VariableNode "x") demand-node)))
)

(define-public (psi-get-action-rules-typed demand-node effect-type)
"
  Returns a SetLink containing the DefinedSchemaNode atoms that name the
  action-rules for the given demand-node.

  demand-node:
  - A ConceptNode that represents a demand.

  effect-type:
  - A string that describes the effect the particualr action would have on
  the demand value. See `(psi-action-types)` for available options.
"
    ; Check arguments
    (if (not (member effect-type (psi-action-types)))
        (error (string-append "Expected fourth argument to be one of the "
            "action types listed when running `(psi-action-types)`, got: ")
            effect-type))


    ; Adds the action-rule selector and uses it to get the action-rules
    (psi-get-action-rules
        (psi-add-action-rule-selector
            (psi-action-rules-typed-term demand-node effect-type)
            (string-append
                (psi-suffix-str (cog-name demand-node))
                "-" effect-type )
        )
        demand-node)
)

; --------------------------------------------------------------
(define-public (psi-get-action-rules-all demand-node)
"
  Returns a list containing the DefinedSchemaNode atoms that name the
  action-rules for the given demand-node, with the exeception of the Default
  action.

  demand-node:
    - A ConceptNode that represents a demand.
"
    ; NOTE: Not using (ure-rbs-rules demand-node) so as to not include 'Default'
    ; action-types, as this function is used by psi-update-asp.
    ; Could there be a reason one would want to change the default action-rule
    ; at runtime?
    (append-map
        (lambda (x)
            (cog-outgoing-set (psi-get-action-rules-typed demand-node x))
        )
        (psi-action-types)
    )
)

; --------------------------------------------------------------
(define-public (psi-get-action-rules-default)
"
  Returns the default actions for all the defined demands
"
    (append-map
        (lambda (x) (cog-outgoing-set (psi-get-action-rules-typed x "Default")))
        (cog-outgoing-set (psi-get-demands-all)))
)

; --------------------------------------------------------------
(define (psi-action-rule-selector-current-typed)
"
  Returns the DefinedPredicateNode that represent the evaluatable term, that
  checks if an action-rule's effect-type is equal to OpenPsi current goal's
  effect-type, and return True-TruthValue if it is and False-TruthValue if not.
"
    (psi-add-action-rule-selector
        (EvaluationLink
            (GroundedPredicateNode "scm: psi-action-rule-selectable-type?")
            (ListLink
                 (DontExecLink (VariableNode "x"))))
        "current-effect-type")
)

(define (psi-action-rule-selectable-type? dsn)
"
  Returns True-TruthValue if it is of the current psi-current-effect-type, and
  False-TruthValue if not.

  dsn:
  - A DefinedSchemaNode that aliases an action-rule.
"
    ; Chech argument
    (if (not (equal? 'DefinedSchemaNode (cog-type dsn)))
        (error "In procedure psi-action-rule-selectable-type?: "
               " Expected DefinedSchemaNode got: " dsn))

    (if (equal? (psi-action-rule-type dsn) (psi-current-effect-type))
        (stv 1 1)
        (stv 0 1)
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

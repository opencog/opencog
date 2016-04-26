; Copyright (C) 2016 OpenCog Foundation

(use-modules (opencog) (opencog exec))

(load "demand.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define-public (psi-action-selector-pattern)
"
  This returns the StateLink that is used for specifying the action selecting
  evaluatable term.

  A StateLink is used instead of an InheritanceLink because there could only
  be one active action-rule-selector at a time even though there could be
  multiple possible action-rule-selectors. And this enables dynamically
  changing the action-rule-selector through learning.
"
    (StateLink
        (ConceptNode (string-append (psi-prefix-str) "action-selector"))
        (VariableNode "$dpn")
    )
)

; --------------------------------------------------------------
(define-public (psi-action-selector-set! dsn)
"
  Sets the given DefinedSchemaNode to be used for selecting actions.

  dsn:
  - The DefinedSchemaNode that represents the executable-term used for
    selecting the psi-rules that should have their actions and goals executed.
"
    ; Check arguments
    (if (not (equal? (cog-type dsn) 'DefinedSchemaNode))
        (error "Expected DefinedSchemaNode got: " dsn))

    (StateLink
        (ConceptNode (string-append (psi-prefix-str) "action-selector"))
        dsn
    )
)

; --------------------------------------------------------------
(define-public (psi-add-action-selector exec-term name)
"
  Returns the DefinedSchemaNode that represents the executable term
  after defining it as an openpsi action-selector.

  exec-term:
  - An evaluatable term.

  name:
  -  A string for naming the action-rule-selector. The name will be prefixed
     by the following string `OpenPsi: action-rule-selector-`.
"
    ; Check arguments
    (if (not (string? name))
        (error "Expected second argument to be a string, got: " name))

    ; TODO: Add checks to ensure the exec-term argument is actually evaluatable
    (let* ((z-name (string-append
                        (psi-prefix-str) " action-selector-" name))
           (selector-dsn (cog-node 'DefinedSchemaNode z-name)))
       (if (null? selector-dsn)
           (begin
               (set! selector-dsn (DefinedSchemaNode z-name))
               (DefineLink selector-dsn exec-term)

                (EvaluationLink
                    (PredicateNode "action-selector-for")
                    (ListLink selector-dsn (ConceptNode psi-prefix-str)))

                selector-dsn
           )

           selector-dsn
       )
    )
)

(define-public (psi-get-action-selector)
"
  Returns a list containing the user-defined action-selector.
"
    (cog-outgoing-set (cog-execute!
        (GetLink (psi-action-selector-pattern))))
)

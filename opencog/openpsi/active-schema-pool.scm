; Copyright (C) 2015 OpenCog Foundation

(use-modules (opencog))
(use-modules (opencog rule-engine))

(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define-public (psi-asp)
"
  Create the active-schema-pool as a URE rulebase and return the node
  representing it.
"
    (let ((asp (ConceptNode (string-append (psi-prefix-str) "asp"))))

        (ure-define-rbs asp 1)

        ; Load all default actions because they should always run. If they
        ; aren't always.
        (if (null? (ure-rbs-rules asp))
            (map (lambda (x) (MemberLink x asp)) (psi-get-action-rules-default)))

        asp
    )
)

; --------------------------------------------------------------
(define-public (psi-step)
"
  The main function that steps OpenPsi active-schema-pool(asp). The asp
  is a rulebase, that is modified depending on the demand-values, on every
  cogserver cycle.

  Returns a list of results from the step.
"
    ;TODO: Move logic to atomese, so as to simply life for everyone.
    (let* ((asp (psi-asp))
         (result (cog-fc (SetLink) asp (SetLink)))
         (result-list (cog-outgoing-set result)))

         (cog-delete result)
         result-list
    )
)

; --------------------------------------------------------------
(define-public (psi-update-asp asp action-rules)
"
  It modifies the member action-rules of OpenPsi's active-schema-pool(asp),
  by removing all action-rules that are members of all the known demand
  rule-bases, with the exception of default-actions, from the asp and replaces
  them by the list of actions passed.

  If the action-rule list passed is empty the asp isn't modified, because the
  policy is that no change occurs without it being explicitly specified. This
  doesn't and must not check what goal is selected.

  asp:
  - The node for the active-schema-pool.

  action-rules:
  - A list of action-rule nodes. The nodes are the alias nodes for the
    action-rules.
"
    (define (remove-node node) (cog-delete-recursive (MemberLink node asp)))
    (define (add-node node) (begin (MemberLink node asp) node))

    (let* ((current-actions (ure-rbs-rules asp))
           (actions-to-keep (psi-get-action-rules-default))
           (actions-to-add
                (lset-difference equal? action-rules current-actions))
           (final-asp (lset-union equal? actions-to-keep action-rules)))

           ; Remove actions except those that should be kept and those that
           ; are to be added.
           (par-map
                (lambda (x)
                    (if (member x final-asp)
                        x
                        (remove-node x))
                )
                current-actions)

           ; Add the actions that are not member of the asp.
           (par-map add-node actions-to-add)
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-pattern)
"
  This returns the StateLink that is used for specifying the goal selecting
  evaluatable term.

  A StateLink is used instead of an InheritanceLink because there could only
  be one active goal-selector at a time eventhough there could be multiple
  possible goal-selectors. And this enables dynamically changing the
  goal-selector through learning.
"
    (StateLink
        (ConceptNode (string-append (psi-prefix-str) "goal-selector"))
        (VariableNode "$dpn")
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-set! dpn)
"
  Sets the given DefinedPredicateNode to be used for selecting goals.

  dpn:
  - The DefinedPredicateNode that represents the evaluatable-term used for
    selecting demand to be a goal.
"
    ; Check arguments
    (if (not (equal? (cog-type dpn) 'DefinedPredicateNode))
        (error "Expected DefinedPredicateNode got: " dpn))
    (cog-execute!
        (PutLink
            (psi-goal-selector-pattern)
            dpn)
    )
)

; --------------------------------------------------------------
(define (psi-add-goal-selector eval-term effect-type name)
"
  Returns the DefinedPredicateNode that represents the evaluatable term
  after defining it as an opencog goal-selector.

  eval-term:
  - An evaluatable term.

  effect-type:
  - A string that describes the effect the particualr action would have on
    the demand value. See `(psi-action-types)` for available options.

  name:
  -  A string for naming the goal selector. `OpenPsi: goal-selector-`
     will be prefixed to the name.
"
    ; Check arguments
    (if (not (string? name))
        (error "Expected third argument to be a string, got: " name))
    (if (not (member effect-type (psi-action-types)))
        (error (string-append "Expected second argument to be one of the "
            "action types listed when running `(psi-action-types)`, got: ")
            effect-type))

    ; TODO: Add checks to ensure the eval-term argument is actually evaluatable
    (let* ((z-name (string-append (psi-prefix-str) "goal-selector-" name))
           (goal-selector-dpn (DefinedPredicateNode z-name)))
        ; This must be first so as to check if a DefinedPredicateNode of the
        ; same name is already defined.
        (DefineLink goal-selector-dpn eval-term)

        (EvaluationLink
            (PredicateNode "selects-for-effect-type")
            (ListLink
                goal-selector-dpn
                (ConceptNode (string-append (psi-prefix-str) effect-type))))

        goal-selector-dpn
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-effect-type dpn)
"
  Returns a node representing the effect type of the given goal-selector.

  dpn:
  - The DefinedPredicateNode that represents the evaluatable-term used for
    selecting demand to be a goal.
"
    ; Check arguments
    (if (not (equal? (cog-type dpn) 'DefinedPredicateNode))
        (error "Expected DefinedPredicateNode got: " dpn))

    ; See https://github.com/opencog/atomspace/issues/646 why this doesn't
    ; work
    ;(GetLink
    ;    (TypedVariableLink
    ;        (VariableNode "$effect")
    ;        (TypeNode "ConceptNode"))
    ;        (EvaluationLink
    ;            (PredicateNode "selects-for-effect-type")
    ;            (ListLink
    ;                (QuoteLink dpn)
    ;                (VariableNode "$effect")))
    ;)

    ; NOTE: Assuming the dpn isn't part of any other similar pattern.
    (car (cog-chase-link 'ListLink 'ConceptNode dpn))
)

; --------------------------------------------------------------
(define (psi-select-random-goal)
"
  Returns the StateLink representing the goal. Goal are defined as demands
  choosen for either increase or decrease in their demand values. For example,

   (StateLink
       (Node (string-append (psi-prefix-str) \"action-on-demand\"))
       (ListLink
           (ConceptNode (string-append (psi-prefix-str) \"Increase\"))
           (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))
  or

   (StateLink
       (Node (string-append (psi-prefix-str) \"action-on-demand\"))
       (ListLink
           (ConceptNode (string-append (psi-prefix-str) \"Decrease\"))
           (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

  The StateLink(aka goal) is the means for signaling what type of actions
  should be selected.
"
    (define (set-goal a-demand effect-type)
       (StateLink
           (Node (string-append (psi-prefix-str) "action-on-demand"))
           (ListLink effect-type a-demand))
    )

    (define (get-goal-selector)
        (let ((goal-selector (cog-outgoing-set (cog-execute!
                    (GetLink (psi-goal-selector-pattern))))))
            (if (null? goal-selector)
                (error "A goal-selector hasn't been set for OpenPsi.")
                (car goal-selector)
            )
        )
    )

    (let* ((goal-selector (get-goal-selector))
           (demands (psi-get-demands goal-selector))
           (effect (psi-goal-selector-effect-type goal-selector)))

       ; If there are no demands that satisfy the condition then choose one
        (if (null? (cog-outgoing-set demands))
            (set-goal
                (cog-execute! (RandomChoiceLink (psi-get-demands-all)))
                (ConceptNode (string-append (psi-prefix-str) "Default")))
            (set-goal (cog-execute! (RandomChoiceLink demands)) effect)
        )
    )
)

; --------------------------------------------------------------
(define (psi-goal-selector-maximize threshold)
"
  Sets the goal by randomly selecting a demand for maximization, should its demand-value be below the given threshold.

  threshold:
  - The boundary of the demand-value below which a demand will be chosen.
"
    (psi-add-goal-selector
        (psi-demand-value-term< threshold) "Increase" "maximize")
)

; --------------------------------------------------------------
(define (psi-select-action-rules)
"
  Selects all actions of current effect type and update the psi-asp.
"
    ;TODO Use psi-select-action-rules, and port as much as possible to atomese.
    (let ((goal (psi-current-goal))
          (effect-type (psi-current-effect-type))
          (asp (psi-asp)))

        ; If default effect-type then add only the default actions.
        (if (equal? effect-type "Default")
            (psi-update-asp  asp (psi-get-action-rules-default))
            (psi-update-asp
                asp (cog-outgoing-set (psi-get-action-rules goal effect-type)))
        )
    )
)

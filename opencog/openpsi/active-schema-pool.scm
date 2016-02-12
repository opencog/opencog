; Copyright (C) 2015 OpenCog Foundation

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
(define (psi-set-goal demand-node effect-type)
"
  Set goal for the action-selector.

  demand-node:
  - The node for the demand choosen to be a goal.

  effect-type:
  - The kind of effect the actions should have for being selected by the
    action-selector.
"
    (define (choose-demand)
        (if (null? demand-node)
            ; NOTE: Remember to deal with the defaults when moving to
            ; psi-select-goal.
            (random-select (psi-get-demands))
            demand-node))

    (let ((z-demand (choose-demand)))
        (StateLink
            (Node (string-append (psi-prefix-str) "action-on-demand"))
            (ListLink
                (ConceptNode (string-append (psi-prefix-str) effect-type))
                z-demand))

        z-demand
    )
)

(define (psi-goal-selector)
"
  This returns the DefinedPredicateNode that specifies tha GroundedPredicateNode
  that is used for goal selecting.
"
    (DefinedPredicateNode (string-append (psi-prefix-str) "goal-selector"))
)

; --------------------------------------------------------------
(define (psi-goal-selector-set! gpn)
"
  Specifies the given GroundedPredicateNode to be used for selecting goals.

  gpn:
  - The GroundedPredicateNode that is used for selecting demand to be a goal.
    The scheme or python function used for selecting the goals is expected to
    only take the node representing the demand. There are no checks to enforce
    the type of GroundedPredicateNode, for now.

"
    (StateLink
        (psi-goal-selector)
        gpn)
    (psi-goal-selector)
)

; --------------------------------------------------------------
(define (psi-goal-random-maximize threshold)
"
  Sets the goal by randomly selecting a demand for maximization, should its demand-value be below the given threshold.

  threshold:
  - The boundary of the demand-value below which a demand will be chosen.
"

    (define (select-demand x) (< (tv-mean (cog-tv x)) threshold))
    (let* ((demand (random-select (filter select-demand (psi-get-demands)))))

        ; If there are no demands that satisfy the condition then choose one
        (if (null? demand)
            (psi-set-goal demand "Default")
            (psi-set-goal demand "Increase")
        )
    )
)

; --------------------------------------------------------------
(define (psi-action-rule-select)
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
            (psi-update-asp  asp (psi-get-action-rules goal effect-type))
        )
    )
)

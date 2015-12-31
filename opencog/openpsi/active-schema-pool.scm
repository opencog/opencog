; Copyright (C) 2015 OpenCog Foundation

(use-modules (opencog rule-engine))

(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define-public (psi-asp)
"
  Create the active-schema-pool as a URE rulebase and return the ConceptNode
  representing it.
"
    (let ((asp (ConceptNode (string-append (psi-prefix-str) "asp"))))

        (ure-define-rbs asp 1)

        ; Load all default actions because they should always run. If they
        ; aren't always.
        (if (null? (ure-rbs-rules asp))
            (ure-add-rules asp (map (lambda (x) (cons x 1)) (psi-get-actions-default))))

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
(define-public (psi-update-asp asp actions)
"
  It modifies the member action-rules of OpenPsi's active-schema-pool(asp),
  by removing all actions that are members of the known demand rule-bases,
  with the exception of default-actions, from the asp and replaces them by the
  list of actions passed.

  If the action list passed is empty the asp isn't modified, because the policy
  is that no change occurs without it being explicitly specified. This doesn't
  and must not check to what goal is selected.

  asp:
  - The ConceptNode for the active-schema-pool.

  actions:
  - A list of actions nodes. The nodes are the alias nodes for the actions.
"
    (define (remove-node node) (cog-delete-recursive (MemberLink node asp)))
    (define (add-node node) (begin (MemberLink node asp) node))

    (let* ((current-actions (ure-rbs-rules asp))
           (actions-to-keep (psi-get-actions-default))
           (actions-to-add
                (lset-difference equal? actions current-actions))
           (final-asp (lset-union equal? actions-to-keep actions)))

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
(define (psi-set-goal demand effect)
"
  Set goal for the action-selector.

  demand:
  - The demand choosen to be a goal.

  effect:
  - The kind of effect the actions should have for being selected by the
    action-selector.
"
    (define (choose-demand)
        (if (null? demand)
            ; XXX Remeber to deal with the defaults when moving to
            ; psi-select-goal.
            (random-select (psi-get-demands))
            demand))

    (let ((z-demand (choose-demand)))
        (StateLink
            (Node (string-append (psi-prefix-str) "action-on-demand"))
            (ListLink
                (ConceptNode (string-append (psi-prefix-str) effect))
                z-demand))

        z-demand
    )
)

; --------------------------------------------------------------
(define (psi-goal-random-maximize threshold)
"
  Sets the goal by randomly selecting a demand for maximization, should its demand-value below the given threshold.

  threshold:
  - The boundary demand-value below which a demand will be choosen.
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
(define (psi-action-select)
"
  Selects all actions of current effect type and update the psi-asp.
"
    ;TODO Use psi-select-actions, and port as much as possible to atomese.
    (let ((goal (psi-current-goal))
          (effect-type (psi-current-effect-type))
          (asp (psi-asp)))

        ; If default effect-type then add only the default actions.
        (if (equal? effect-type "Default")
            (psi-update-asp  asp (psi-get-actions-default))
            (psi-update-asp  asp (psi-get-actions goal effect-type))
        )
    )
)

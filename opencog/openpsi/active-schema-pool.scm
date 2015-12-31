; Copyright (C) 2015 OpenCog Foundation

(load-from-path "openpsi/demand.scm")
(load-from-path "openpsi/utilities.scm")

; --------------------------------------------------------------
(define-public (psi-asp)
"
  Create the active-schema-pool as a URE rulebase and return the ConceptNode
  representing it.
"
    (let ((asp (ConceptNode (string-append (psi-prefix-str) "asp"))))
        ; Define OpenPsi active-schema-pool as the ure rulebase. The iternation
        ; has no effect as all rules are run once per cycle by the agent.
        (ure-define-rbs asp 1)
    )
)

; --------------------------------------------------------------
(define-public (psi-run)
"
  The main function that runs OpenPsi active-schema-pool. The active-schema-pool
  is a rulebase, that is modified depending on the demand-values, on every
  cogserver cycle.
"

    (cog-fc (SetLink) (psi-asp) (SetLink))
)

; --------------------------------------------------------------
(define-public (psi-update-asp asp actions)
"
  It modifies the member action-rules of OpenPsi's active-schema-pool(asp),
  by removing all actions that are members of the known demand rule-bases,
  with the exception of default-actions, from the asp and replaces them by the
  list of actions passed.

  asp:
  - The ConceptNode for the active-schema-pool.

  actions:
  - A list of actions nodes. The nodes are the alias nodes for the actions.
"
    (define (remove-node node)
        (cog-delete (MemberLink node asp)))
    (define (add-node) node
        (MemberLink node asp))

    (let* ((current-actions (ure-rbs-rules asp))
           (all-actions
               ;TODO update psi-get-all-actions  on replacing with appropriate
               ; filter
               (list-merge (par-map psi-get-all-actions (psi-get-demands))))
           ; Because there might be rules added that aren't member of any demand
           ; like the default/in-born rule that updates the rules.
           (actions-to-keep (lset-difference current-actions all-actions)))

           ; Remove actions except those that should be kept and those that
           ; are to be added from the asp
           (par-map
                (lambda (x)
                    (if (or (member x actions-to-keep) (member x actions))
                        x
                        (remove-node x))
                )
                current-actions)

            ; Add the actions that are not member of the asp.
            (par-mpa add-node actions)
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
    (StateLink
        (Node (string-append (psi-prefix-str) "action-on-demand"))
        (ListLink
            (ConceptNode (string-append (psi-prefix-str) effect))
            demand))
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
        (psi-set-goal demand "Increase")
    )
)

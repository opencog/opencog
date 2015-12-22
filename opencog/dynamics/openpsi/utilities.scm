; Copyright (C) 2015 OpenCog Foundation

; --------------------------------------------------------------
; Helper Functions
; --------------------------------------------------------------
(define (psi-clean-demand-gets set-link)
"
  Returns a list aftering removing the 'SetLink + 'ListLink + 'NumberNode
  from result of `cog-execute!` of 'GetLink wrapping psi-demand-pattern with or
  without additional clauses.

  set-link:
    - A 'SetLink that is a result of running `cog-execute!` of 'GetLink
      wrapping psi-demand-pattern.
"
; TODO: Make this generic for pattern gets
    (define (list-merge list-of-list) (fold append '() list-of-list))
    (define (is-nn? x) (equal? (cog-type x) 'NumberNode))
    (remove! is-nn?
        (list-merge (map cog-outgoing-set (cog-outgoing-set set-link))))
)

; --------------------------------------------------------------
(define (psi-prefix-str)
"
  Returns the string used as a prefix to all OpenPsi realted atom definition
"
 "OpenPsi: "
)

(define (psi-suffix-str a-string)
"
  Returns the suffix of that follows `psi-prefix-str` sub-string.

  a-string:
    - a string that should have the`psi-prefix-str`

"
    (let ((z-match (string-match (psi-prefix-str) a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" (psi-prefix-str) "\"") )
        )
    )
)

; --------------------------------------------------------------
; Functions for OpenPsi active-schema-pool
; --------------------------------------------------------------
(define (psi-run)
"
  The main function that runs OpenPsi active-schema-pool. The active-schema-pool
  is a rulebase, that is modified depending on the demand-values, on every
  cogserver cycle.
"

    (cog-fc (SetLink)
        (ConceptNode (string-append (psi-prefix-str) "active-schema-pool"))
        (SetLink))
)

; --------------------------------------------------------------
(define (psi-select-goal gpn)
"
  Goal are defined as demands choosen for either increase or decrease in
  their demand values. The choice for being the current-goal is made by pattern
  matching over the demands by using the GroundedPredicateNode passed as a
  constraint.

  The GroundedPredicateNode passed should tag the demands for increase or
  decrease by evaluating the demands that satisfy the conditions set through
  its goal-selection function. The function should tag the goal representing
  atom (the demand-ConceptNode) for being increased or decreased.

  gpn:
    - GroundedPredicateNode that names an goal-selection function that will
      choose the demands . Its function should take a single demand-ConceptNode
      and return True-TruthValue `(stv 1 1)`  or False-TruthValue `(stv 0 1)`
      in addition to tagging the demand-ConceptNodes as either,

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Increase\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Decrease\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      The tags are necessary because, that is the means for signaling what type
      of actions should be taken, in effect it is the demand-goal. For example,
      if the action-on-demand is Increase, then only the actions of type
      Increase would be choosen.

"

    ; XXX: Should there be weight b/n the different demand-goals? For now a
    ; a random choice of demands is assumed. In subsequent steps. The
    ; demand-value could possibly be used for that.
    (define (get-demand) (psi-clean-demand-gets (cog-execute!
        (GetLink
            (VariableList
                (assoc-ref (psi-demand-pattern) "var"))
            (AndLink
                (assoc-ref (psi-demand-pattern) "pat")
                (EvaluationLink
                    gpn
                    (ListLink (VariableNode "Demand"))))))))

    ; check arguments
    (if (not (equal? (cog-type gpn) 'GroundedPredicateNode))
        (error "Expected DefinedPredicateNode got: " gpn))

    (get-demand)
)


; --------------------------------------------------------------
(define (psi-get-current-goal)
"
  Returns the demand-ConceptNode that has been choosen for action presently.
"
   (define (get-psi-goal) (cog-execute!
      (GetLink
        (TypedVariableLink
            (VariableNode "demand")
            (TypeNode "ConceptNode"))
        (AndLink
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ChoiceLink
                    (ListLink
                        (ConceptNode
                            (string-append (psi-prefix-str) "Decrease"))
                        (VariableNode "demand"))
                    (ListLink
                        (ConceptNode
                            (string-append (psi-prefix-str) "Increase"))
                        (VariableNode "demand"))))
            (EvaluationLink ; Act only if their is such a demand.
                (GroundedPredicateNode "scm: psi-demand?")
                (ListLink
                    (VariableNode "demand")))))
    ))

    (let* ((set-link (get-psi-goal))
          (result (car (cog-outgoing-set set-link)))
          )
          (cog-delete set-link)
          result
    )
)

; --------------------------------------------------------------
(define (psi-get-current-action-type)
"
  This returns a node of the type of effect that the current-goal have.
"
   (define (get-psi-action-type) (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "effect-type")
                (TypeNode "ConceptNode"))
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ListLink
                    (VariableNode "effect-type")
                    (psi-get-current-goal))))
    ))

    (let* ((set-link (get-psi-action-type))
          (result (car (cog-outgoing-set set-link)))
          )
          (cog-delete set-link)
          result
    )
)

; --------------------------------------------------------------
(define (psi-select-actions demand-node gpn)
"
  Select the actions that should be added to the active-schema-pool depending
  on the present goal, by using the plan choosen by the GroundedPredicateNode.

  XXX should it modify the member action-rules of the active-schema-pool.

  demand-node:
    - A ConceptNode that represents a demand

  gpn:
   - GroundedPredicateNode that refers to a function that checks the actions
     for constraints.
"
    ;TODO: I think the planner is kind of a behavior tree genrator (assuming
    ; there is no change of a preset plan) .URE's random selection policy
    ; isn't being used now thus each plan is in effect a single action choosen,
    ; this has to be improved but is good for starters.


    ;(psi-get-actions demand-node gpn)
    (display "WIP")
)

; --------------------------------------------------------------
(define (psi-update-asp demand-node gpn)
"
  It modifies the member action-rules of the active-schema-pool.
"
;not sure if should be part of psi-select-actions, why separate them?
    ;(let ((choosen-actions (psi-select-actions demand-node gpn)))
    ;    ()
    ;)

(display "WIP")
)

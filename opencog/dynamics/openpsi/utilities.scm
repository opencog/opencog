(use-modules (rnrs sorting)) ; needed for sorting demands by their values.
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
    (define (list-merge list-of-list) (fold append '() list-of-list))
    (define (is-nn? x) (equal? (cog-type x) 'NumberNode))
    (remove! is-nn?
        (list-merge (map cog-outgoing-set (cog-outgoing-set set-link))))
)

; --------------------------------------------------------------
; Functions for OpenPsi Demands
; --------------------------------------------------------------
(define (psi-prefix-str)
"
  returns the string used as a prefix to all OpenPsi realted atom definition
"
 "OpenPsi: "
)

; --------------------------------------------------------------
(define (psi-demand-pattern)
"
  Returns an alist used to define the psi demand pattern. The key strings are,
  - 'var': its value is a list containing the VariableNodes and their type
           restrictions.
  - 'pat': its value is a DefinedPredicateNode that is associated with the
           demand pattern.
"
    (define z-alist (acons "var" (list
        (TypedVariableLink
            (VariableNode "Demand")
            (TypeNode "ConceptNode")
        )
        (TypedVariableLink
            (VariableNode "min_acceptable_value")
            (TypeNode "NumberNode")
        )
        (TypedVariableLink
            (VariableNode "max_acceptable_value")
            (TypeNode "NumberNode")
        ))
        '()))
    (define dpn (DefinedPredicateNode
        (string-append (psi-prefix-str) "demand-pattern")))

    (DefineLink
        dpn
        (PresentLink
            (InheritanceLink
                ; the strength of the stv is the demand value
                (VariableNode "Demand")
                (ConceptNode (string-append (psi-prefix-str) "Demand")))
            ; This is equivalent to an in-born drive/goal. When
            ; the value goes out of range then urge occurs
            (EvaluationLink
                (PredicateNode "must_have_value_within")
                (ListLink
                    (VariableNode "Demand")
                    (VariableNode "min_acceptable_value")
                    (VariableNode "max_acceptable_value")))
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "acts-on"))
                (ListLink
                    (GroundedSchemaNode "scm: psi-demand-updater")
                    (VariableNode "Demand")))))
    (acons "pat" dpn z-alist)
)

; --------------------------------------------------------------
(define (psi-get-demands)
"
  Returns a list containing the ConceptNode that carry the demand-value. The
  strength of their stv is the demand value.
"
    (psi-clean-demand-gets (cog-execute!
        (GetLink
            (VariableList (assoc-ref (psi-demand-pattern) "var"))
            (assoc-ref (psi-demand-pattern) "pat"))))
)

; --------------------------------------------------------------
(define (define-psi-demand  demand-name default-value min-value max-value)
"
  Define an OpenPsi demand. By default an in-born drive/action that aims to
  maintain the goal of keeping the demand-value within the given range is
  defined.

  demand-name:
    - The name of the demand that is created.

  default-value:
    - The initial demand-value. This is the strength of the demand's
      ConceptNode stv. The confidence of the stv is always 1.

  min-value:
    - The minimum acceptable demand-value. On going lower tha

  max-value:
    - The maximum acceptable demand-value.

"
    (let ((demand (string-append (psi-prefix-str) demand-name)))
        (list
            (InheritanceLink
                (ConceptNode demand (stv default-value 1))
                (ConceptNode (string-append (psi-prefix-str) "Demand"))
            )

            ; This is the goal of the demand
            (EvaluationLink
                (PredicateNode "must_have_value_within")
                (ListLink
                    (ConceptNode demand)
                    (NumberNode min-value)
                    (NumberNode max-value)
                )
            )

            ; This specifies the default action that each psi-demand must have.
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "acts-on"))
                (ListLink
                    (GroundedSchemaNode "scm: psi-demand-updater")
                    (ConceptNode demand)
                )
            )
        )
    )
)

; --------------------------------------------------------------
(define (psi-demand? atom)
"
  Checks whether an atom is the ConceptNode that satisfies the pattern used
  to define an OpenPsi demand. Returns True-TruthValue `(stv 1 1)` if it is
  and False-TruthValue `(stv 0 1)` if it isn't.

  atom:
  - The atom that is being checked to see if it is the Node that represents
    a demand type.
"
    (define demand-names (map cog-name (psi-get-demands)))
    (if (and (member (cog-name atom) demand-names)
             (equal? (cog-type atom) 'ConceptNode))
        (stv 1 1)
        (stv 0 1)
    )
)

; --------------------------------------------------------------
(define (psi-lowest-demand? atom)
"
  Returns #t if the atom passed is a demand that has the lowest demand-value.

  atom:
  - The atom that is being checked to see if it is the Node that represents
    a demand type, with a lowest demand-value.
"
    ; check if atom is a demand-node
    (if (equal? (stv 0 1) (psi-demand? atom))
        (error "Expected argument to be a demand-node, got: " atom))

    (let ((atom-strength (tv-mean (cog-tv atom)))
          (lowest-demand-value (car (list-sort < (delete-duplicates
              (map (lambda (x) (tv-mean (cog-tv x))) (psi-get-demands))))))
         )
         (if (<= atom-strength lowest-demand-value)
            (stv 1 1)
            (stv 0 1)
         )
    )
)

; --------------------------------------------------------------
(define (define-psi-action vars context action demand-name effect-type)
"
  It associates an action and context in which the action has to be taken
  to an OpenPsi-demand. It returns a BindLink, structured as
    (BindLink
        (VariableList (vars))
        (AndLink
            (context)
            (clauses required for linking with the demand named demand-name))
        (action))
  If the action, context, and effect-type have already been defined in the
  atomspace then the previous defined BindLink is returned.

  A single action-rule could only have a either of the effect-types, thus
  changing the effect-type will not have any effect if the action-rule has
  already been defined in the atomspace with a different effect-type.

  vars:
    - A list containing the VariableNodes, and their type restrictions, that
      are part of the context. If there is no type restrictions then pass empty
      list.

  context:
    - A list containing the terms/clauses that should be met for this action
      to be taken. Be careful on how you use Variable naming in the context

  action:
    - The Implicand of the rule. It should be an atom that uses the groundings
      of the context to do something.

  demand-name:
    - The name of the demand that is affected by the execution of the function
      associated with the action.
    - Shouldn't include `psi-prefix-str`
    - It is case sensetive, that is, a demand named `Energy` is not equal
      to a demand named `energy`. If you pass a name of a not defined node,
      since it's unrecognized it won't be run, eventhough a BindLink is
      returned.

  effect-type:
    - A string that describes the effect the particualr action would have on
      the demand value. The only options are `Increase` and `Decrease`.

"
    (define rule-name-prefix
        (string-append (psi-prefix-str) demand-name "-rule-"))
    (define rule-name (string-append rule-name-prefix (random-string 5)))
    (define (demand-node)
        (ConceptNode (string-append (psi-prefix-str) demand-name)))

    (define (rule)
        ; Is function to avoid  insertion into the atomspace if argument check
        ; fails.
        (BindLink
            ; An empty VariableList prevents matchs.
            (if (equal? '() vars)
                '()
                (VariableList vars)
            )
            (AndLink
                context
                (EvaluationLink ; Act only if their is such a demand.
                    (GroundedPredicateNode "scm: psi-demand?")
                    (ListLink (demand-node))))
            action))

    (define (link-with-demand)
        ; For finding the rule. The atom is not part of the rule because if
        ; rule-name is randomely generated, as such unless by chance all the
        ; rules will not much even if the context and Variable types match.
        (EvaluationLink
            (PredicateNode (string-append (psi-prefix-str) effect-type))
            (ListLink
                (Node rule-name)
                (demand-node))))

    ; Check arguments
    (if (not (list? vars))
        (error "Expected first argument to be a list, got: " vars))
    (if (not (list? context))
        (error "Expected second argument to be a list, got: " context))
    (if (not (cog-atom? action))
        (error "Expected third argument to be an atom, got: " action))
    (if (not (member effect-type (list "Increase" "Decrease")))
        (error (string-append "Expected fourth argument to be either"
            "`Increase` or `Decrease` got: ") effect-type))

    ; 1. Get all nodes that define the BindLink as a ure recognizable rule.
    ; 2. Check if the rule has already been defined, and return the rule or
    ;    define the rule if it hasn't, or throw an error if it has been defined
    ;    more than once.
    (let ((node (cog-chase-link 'DefineLink 'Node (rule))))
        (cond ((and (equal? 1 (length node))
                    (string-prefix? rule-name-prefix (cog-name (car node))))
                     (rule))
              ((equal? 0 (length node))
                    (begin (DefineLink (Node rule-name) (rule))
                        (link-with-demand) (rule)))
              (else (error "The rule has been defined multiple times"))
        )
    )
)

; --------------------------------------------------------------
(define (psi-get-actions demand-node effect-type)
"
  Returns a list containing the 'Node atom-type atoms that name the action-rules
  for the given demand-node.

  demand-node:
    - A ConceptNode that represents a demand.

  effect-type:
    - A string that describes the effect the particualr action would have on
      the demand value. The only options are `Increase` and `Decrease`.
"
    ; Check arguments
    (if (not (member effect-type (list "Increase" "Decrease")))
        (error (string-append "Expected second argument to be either"
            "`Increase` or `Decrease` got: ") effect-type))

    (cog-outgoing-set (cog-execute!
        (GetLink
             (TypedVariableLink
                 (VariableNode "x")
                 (TypeNode "Node"))
             (EvaluationLink
                 (PredicateNode (string-append (psi-prefix-str) effect-type))
                 (ListLink
                    (VariableNode "x")
                    demand-node)))
    ))
)

; --------------------------------------------------------------
(define (psi-get-all-actions demand-node)
"
  Returns a list containing the 'Node atom-type atoms that name the action-rules
  for the given demand-node.

  demand-node:
    - A ConceptNode that represents a demand.
"
    (append
        (psi-get-actions demand-node "Increase")
        (psi-get-actions demand-node "Decrease")
    )
)

; --------------------------------------------------------------
; Functions for OpenPsi Modulators
; --------------------------------------------------------------
(define (psi-get-modulators)
"
  Returns a list containging the atoms defining the modulators.
"
    (let* ((psi-modulator (ConceptNode
                            (string-append (psi-prefix-str) "Modulator")))
          (inheritance-list (cog-incoming-set psi-modulator))
         )
         (remove!
             (lambda (x) (equal? (cog-type x) 'VariableNode))
             (par-map cog-get-partner inheritance-list
                 (make-list (length inheritance-list) psi-modulator))
         )
    )
)

; --------------------------------------------------------------
(define (define-psi-modulator modulator-name stimulus-value)
"
  Define an OpenPsi modulator. By default an in-born stimulus is defined.

  modulator-name:
    - The name of the modulator that is created.

  stimulus-value:
    - The initial stimulus level. This is the strength of the modulator's
      ConceptNode stv. The confidence of the stv is always 1.
"
    (let ((modulator (string-append (psi-prefix-str) modulator-name)))
        (list
            (InheritanceLink
                (ConceptNode modulator (stv stimulus-value 1))
                (ConceptNode (string-append (psi-prefix-str) "Modulator"))
            )

            ; This is the default function that each psi-modulator must have.
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "Stimulus"))
                (ListLink
                    (GroundedSchemaNode "scm: psi-modulator-updater")
                    (ConceptNode modulator)
                )
            )
        )
    )
)

; --------------------------------------------------------------
(define (define-psi-stimulus gsn modulator-name)
"
  It associates a stimulus to an OpenPsi-modulator

  gsn:
    - A valid GroundedSchemaNode. Since there is no type checking done, be
      be sure that it is properly defined.

  modulator-name:
    - The name of the modultor that is stimulated by the execution of the
      function associated with the GroundedSchemaNode.
"
    (EvaluationLink
        (PredicateNode (string-append (psi-prefix-str) "Stimulus"))
        (ListLink
            (if (equal? (cog-type gsn) 'GroundedSchemaNode)
                gsn
                (error (string-append "pass GroundedSchemaNode as the 1st "
                     "argument in define-psi-stimulus"))
            )
            (ConceptNode (string-append (psi-prefix-str) modulator-name))
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
  Returns the demand that is being acted upon presently. When a demand is set
  for action it would be used.
"
(display "WIP")

#!
;this doesn't work, seems to be a bug.
   (cog-execute!
      (GetLink
        (TypedVariableLink
            (VariableNode "demand")
            (TypeNode "ConceptNode"))
        (AndLink
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ListLink
                    (ChoiceLink
                        (ConceptNode
                            (string-append (psi-prefix-str) "Decrease"))
                        (ConceptNode
                            (string-append (psi-prefix-str) "Increase")))
                    (VariableNode "demand")))
            (EvaluationLink ; Act only if their is such a demand.
                (GroundedPredicateNode "scm: psi-demand?")
                (ListLink
                    (VariableNode "demand"))))))
!#
)

; --------------------------------------------------------------
(define (psi-select-actions demand-node gpn)
"
  Select the actions that should be added to the active-schema-pool depending
  on the present goal, by using the plan choosen by the GroundedPredicateNode.

  XXX should it modify the member action-rules of the active-schema-pool.

  demand-node:
    - A ConceptNode that represents a demand.

  gpn:
   - GroundedPredicateNode that refers to a function that checks the actions
     for constraints.
"
    ;TODO: I think the planner is kind of a behavior tree genrator (assuming
    ; there is no change of a preset plan) .URE's random selection policy
    ; isn't being used now thus each plan is in effect a single action choosen,
    ; this has to be improved but is good for starters.

; (psi-get-actions (psi-get-current-goal))
(display "WIP")
)

; --------------------------------------------------------------
(define (psi-update-asp demand-node gpn)
"
  It modifies the member action-rules of the active-schema-pool.
"
;not sure if should be part of psi-select-actions, why separate them?
    ;(let ((choosen-actions (psi-select-actions demand-node gpn)))

    ;)

(display "WIP")
)

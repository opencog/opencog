; Copyright (C) 2016 OpenCog Foundation

(use-modules (ice-9 receive))

(use-modules (opencog) (opencog exec) (opencog query))

(load "demand.scm")
(load "rule.scm")
(load "utilities.scm")

; --------------------------------------------------------------
(define-public (psi-set-updater! updater tag-node)
"
  Returns the alias node that represents the updater.

  updater:
  - An evaluatable link/node which when evaluated will updater the values for
    the given demand/modulator.

  tag-node:
  - A demand/modulator node that the updater is being added to.
"
    (psi-set-functionality updater #t tag-node "updater")
)

; --------------------------------------------------------------
(define-public (psi-get-updater tag-node)
"
  Returns a list containing the updater for the given tag-node. Null is returned
  if it doesn't have one.

  tag-node:
  - A demand/modulator node that has the updater.
"
    (psi-get-functionality tag-node "updater")
)

; --------------------------------------------------------------
; This demand used to tag a psi-rule that is being manually controlled.
; It controls the weight of other demands through the updater associated with
; it. It shouldn't be skipped.
; NOTE: This is a hack b/c once the weight of the rules is separated into atom
; or protoatom wrapped in a StateLink then (or somehting like that) then it
; would be easier to update.
(define psi-controller-demand (psi-demand "controller" .000000000))

; --------------------------------------------------------------
(define-public (psi-rule-set-atomese-weight psi-rule weight)
"
  Returns the StateLink that is used to represent the weight of an psi-rule.

  psi-rule:
  - An ImplicationLink whose weight is going to be modified.

  weight:
  - A number signifiying the weight of the rule.
"
    (StateLink ; FIXME should use AtTimeLink
        (ListLink
            (psi-rule-alias psi-rule)
            (ConceptNode (string-append psi-prefix-str "weight")))
        (NumberNode weight))
)

; --------------------------------------------------------------
(define-public (psi-set-controlled-rule psi-rule)
"
  Specify that the psi-rule is to be controled. Controlling means modifying the
  weight of the rule, thus affecting the likelyhood of it being choosen.

  psi-rule:
  - An ImplicationLink whose weight is going to be modified.
"
    (MemberLink psi-rule psi-controller-demand)

    (psi-rule-set-atomese-weight psi-rule (tv-mean (cog-tv psi-rule)))

    psi-rule
)

; --------------------------------------------------------------
(define-public (psi-get-controlled-rules)
"
  Returns a list of all the rules that are going to be controlled
"
    (cog-outgoing-set
        (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "controled-rule")
                (TypeNode "ImplicationLink"))
            (MemberLink (Variable "controled-rule") psi-controller-demand))))
)

; --------------------------------------------------------------
(define-public (psi-rule-atomese-weight psi-rule)
"
  Returns a list with a NumberNode that has the weight of the given psi-rule if
  the weight is represented in atomese, if not it returns `null`.

  psi-rule:
  - An ImplicationLink whose weight is going to be modified.
"
    (cog-outgoing-set (cog-bind
        (BindLink
            (VariableList
                (TypedVariableLink
                    (VariableNode "psi-rule-weight")
                    (TypeNode "NumberNode"))
                (TypedVariableLink
                    (VariableNode "psi-rule-alias")
                    (TypeNode "ConceptNode")))
            (AndLink
                (StateLink ; FIXME should use AtTimeLink
                    (ListLink
                        (VariableNode "psi-rule-alias")
                        (ConceptNode (string-append psi-prefix-str "weight")))
                    (VariableNode "psi-rule-weight"))
                (EvaluationLink
                    psi-rule-name-predicate-node
                    (ListLink
                        (QuoteLink psi-rule)
                        (VariableNode "psi-rule-alias"))))
            (VariableNode "psi-rule-weight"))
    ))
)

; --------------------------------------------------------------
(define-public (psi-controller-update-weights)
"
  This gets all the psi-rules that inherit from the psi-control-node & update
  their weight.
"
    (define (update-weight-from-atomese-weight rule)
        (let ((result (psi-rule-atomese-weight rule)))
            (if (not (null? result))
                (cog-set-tv! rule
                    (stv (string->number (cog-name (car result)))
                         (tv-conf (cog-tv rule))))
            )
        ))

    (map update-weight-from-atomese-weight
        (psi-get-rules psi-controller-demand))
)

; --------------------------------------------------------------
(define-public (psi-rule-disable rule-alias psi-rule-list)
"
  Disable the psi-rule which is aliased as `rule-alias`. A disabled rule is
  one with weight of zero.

  rule-alias:
  - string used for a psi-rule alias.

  psi-rule-list:
  - a list with psi-rules.
"
    (receive (filtered other)
        (psi-partition-rule-with-alias rule-alias psi-rule-list)
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.0))
            filtered)
    )
)

; --------------------------------------------------------------
(define-public (psi-rule-enable rule-alias psi-rule-list)
"
  Disable the psi-rule which is aliased as `rule-alias`. An enabled rule is
  one with weight greater than zero. Here the weight is set to 0.9

  rule-alias:
  - string used for a psi-rule alias.

  psi-rule-list:
  - a list with psi-rules.
"
    (receive (filtered other)
        (psi-partition-rule-with-alias rule-alias psi-rule-list)
        (map
            (lambda (psi-rule) (psi-rule-set-atomese-weight psi-rule 0.9))
            filtered)
    )
)

; --------------------------------------------------------------
; Thought of using the updater in the psi-step but got to add additional
; functions for filtering by tags/demands
;
;(define controller-updater
;    (DefinedPredicateNode (string-append psi-prefix-str "controler")))
;
;(DefineLink
;    controller-updater
;    (EvaluationLink
;        (GroundedPredicateNode "scm: psi-controller-update-weights")
;        (ListLink))
;)
;
;(psi-set-updater! controller-updater psi-controller-demand)

; --------------------------------------------------------------
; Skip controller-demand as it is only used to update weight of the rules that
; are associated with it.
(psi-demand-skip psi-controller-demand)
(psi-reset-valid-demand-cache)

;(load "utilities.scm")

(define psi-active-schema-pool
    (ConceptNode (string-append (psi-prefix-str) "active-schema-pool")))

(InheritanceLink  ; Defining a rule base
    psi-active-schema-pool
    (ConceptNode "URE"))

(ExecutionLink
    (SchemaNode "URE:maximum-iterations")
    psi-active-schema-pool
    (NumberNode "10"))

; Reminder : 1. When ure has the capability to forward chain over only a subset
; of atoms (either defined by ecan or just passed to it) then if a rule doesn't
; apply it may be removed from asm.
;            2. openpsi can be thought of one of the levels descibed in
; https://en.wikipedia.org/wiki/Subsumption_architecture. The dialogue system
; with it's own pool another. Then there will be a need for a central action
; selection that ensures that the individual action-selectors associated with
; each
; pool don't choose an action that is in conflict with that found in another.
; The relations between actions could be hard coded or learnt.
;            3. A rule could be defined to affect a single demand/modulator
; value. For e.g. a rule that tries

(MemberLink (stv 1 1)
    (Node "psi-demand-updater-rule")
    psi-active-schema-pool)

(MemberLink (stv 1 1)
    (Node "psi-modulator-updater-rule")
    psi-active-schema-pool)

(DefineLink
    (ConceptNode
        (string-append (psi-prefix-str) "default-asp-goal-selector-gpn"))
    (GroundedPredicateNode "scm: psi-lowest-demand?"))

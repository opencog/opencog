(define openpsi-active-schema-pool (ConceptNode "OpenPsi: active-schema-pool"))

(ExecutionLink
    (SchemaNode "URE:maximum-iterations")
    (ConceptNode "OpenPsi: active-schema-pool")
    (NumberNode "100")
)

; Reminder : 1. When ure has the capability to forward chain over only a subset
; of atoms (either defined by ecan or just passed to it) then if a rule doesn't
; apply it may be removed from asm.
;            2. openpsi can be thought of one of the levels descibed in
; https://en.wikipedia.org/wiki/Subsumption_architecture. The dialogue system
; with it's own pool another. Then there will be a need for a central action
; selection that ensures that the invidual action-selectors associated with each
; pool don't choose an action that is in conflict with that found in another.
; The relations between actions could be hard coded or learnt. 

; The action-selection-agent/planner will choose the rules that are member of
; the active schema pool for execution with the exception of the
; psi-demand-updater-rule & psi-modulator-updater-rule
(MemberLink (stv 1 1)
    psi-demand-updater-rule
    (ConceptNode "OpenPsi: active-schema-pool")
)

(MemberLink (stv 1 1)
    psi-modulator-updater-rule
    (ConceptNode "OpenPsi: active-schema-pool")
)

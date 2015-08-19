; Integrity psi-demand definition
(InheritanceLink
    ; The strength of the stv is the demand-value, & confidence is always 1.
    (ConceptNode "OpenPsi: Integrity" (stv .6 1))
    (ConceptNode "OpenPsi: Demand")
)

(EvaluationLink
    (PredicateNode "must_have_value_within")
    (ListLink
        (ConceptNode "OpenPsi: Integrity")
        (NumberNode .8)
        (NumberNode 1.)
    )
)

; actions that can affect on the psi-demand Competence.
; XXX: Why have mutlitple scheme functions that act on a demand without defining
; a rule. -> This is just part of the psi module atomese api.
; TODO: Should this be changed to ExecutionLink that specify the input of the
; GSN using SignatureLink? that way the rule defining functions could extract
; the patterns to search for.
(EvaluationLink
    (PredicateNode "Psi: acts-on")
    (ListLink
        ; This is the default function that each psi-demand must have.
        (GroundedSchemaNode "scm: psi-demand-updater")
        (ConceptNode "OpenPsi: Integrity")
    )
)

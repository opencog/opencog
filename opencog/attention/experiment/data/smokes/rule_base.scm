; Load the rules (use load for relative path w.r.t. to that file)
(load "modus-ponens.scm")
(load "deduction.scm")
(load "smokes_rb.scm")

; Associate the rules to the rule base (with weights, their semantics
; is currently undefined, we might settled with probabilities but it's
; not sure)
(MemberLink
   pln-rule-modus-ponens-name
   (ConceptNode "SMOKES_RB")
)
(MemberLink (stv 1 1)
   pln-rule-deduction-name
   (ConceptNode "SMOKES_RB")
)

(MemberLink (stv 1 1)
   smokes-rule-smokes-name
   (ConceptNode "SMOKES_RB")
)
(MemberLink (stv 1 1)
   smokes-rule-has-cancer-name
   (ConceptNode "SMOKES_RB")
)
(MemberLink (stv 1 1)
   smokes-rule-friends-name
   (ConceptNode "SMOKES_RB")
)

;termination criteria parameters
(ExecutionLink
   (SchemaNode "URE:maximum-iterations")
   (ConceptNode "SMOKES_RB")
   (NumberNode "100")
)

;Attention allocation (set the TV strength to 0 to disable it, 1 to
;enable it)
(EvaluationLink (stv 0 1)
   (PredicateNode "SMOKES_RB:attention-allocation")
   (ConceptNode "SMOKES_RB")
)

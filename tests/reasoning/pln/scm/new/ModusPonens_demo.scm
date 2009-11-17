; For ModusPonensRule (in code: StrictImplicationBreakdownRule),
; using ImplicationBreakdownFormula.
; This Rule only works when the premises are Links, not Nodes; I'm not sure
; there's a good reason for that. The code for the Rule and the Formula are
; both weird; I'm not quite sure what atoms it uses and how. -- JaredW

(define eval_a (EvaluationLink (PredicateNode "A") (stv 0.5 0.8)))   ; premise #2
(define eval_b (EvaluationLink (PredicateNode "B")))                 ; conclusion
(define imp_eva_evb (ImplicationLink eval_a eval_b (stv 0.85 0.79))) ; premise #1


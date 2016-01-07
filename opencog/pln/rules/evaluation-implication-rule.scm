;; =============================================================================
;; Evaluation Implication Rule
;;
;; EvaluationLink
;;   A
;;   B
;; ImplicationLink
;;   A
;;   C
;; |-
;; EvaluationLink
;;   C
;;   B
;; -----------------------------------------------------------------------------
;; Based on Chapter 10: Higher Order Extensional Inference
;; This rule handles some cases of PLN higher order logic that don't
;; require quantifiers

;; ImplicationLink PredicateNode:A PredicateNode:A
;; can be handled trivially using the existing first-order PLN rules.
;; This is where we handle a special case.


(load "formulas.scm")

(define evaluation-implication-rule
    (BindLink
        (VariableList
            (VariableNode "$A")
            (VariableNode "$B")
            (VariableNode "$C"))
        (AndLink
            (EvaluationLink
                (VariableNode "$A")
                (VariableNode "$B"))
            (ImplicationLink
                (VariableNode "$A")
                (VariableNode "$C")))
        (ExecutionOutputLink
            (GroundedSchemaNode "scm: evaluation-implication-formula")
            (ListLink
                (EvaluationLink
                    (VariableNode "$A")
                    (VariableNode "$B"))
                (ImplicationLink
                    (VariableNode "$A")
                    (VariableNode "$C"))
                (EvaluationLink
                    (VariableNode "$C")
                    (VariableNode "$B"))))))

(define (evaluation-implication-formula AB AC CB)
    (define A (gar AB))
    (define B (gdr AB))
    (define C (gdr AC))
    (cog-set-tv!
        CB
        (stv
            (simple-deduction-strength-formula 
                (cog-stv-strength B)
                (cog-stv-strength A)
                (cog-stv-strength C)
                (cog-stv-strength AB)
                (cog-stv-strength AC))
            (*
                (* 0.9 0.9)
                (min
                    (cog-stv-confidence B)
                    (cog-stv-confidence A)
                    (cog-stv-confidence C)
                    (cog-stv-confidence AC)
                    (* 0.9 (cog-stv-confidence AB)))))))

; Name the rule
(define evaluation-implication-rule-name (DefinedSchemaNode "evaluation-implication-rule"))
(DefineLink evaluation-implication-rule-name evaluation-implication-rule)


; Similarity substitution Rule is equivalent to combining that rule with
; Abduction
; 
; Deduction:
;   A does B
;   B causes C
;   |-
;   A causes C
;
; Abduction:
;   A does C
;   A isa B
;   |-
;   B's do C
;
; Implication A B
;   Implication A C (from Eval C A)
;   |- 
;   Abduction Rule
;       Implication B C
;   
; Induction:
;   B causes A
;   B causes C
;   |-
;   A is C
;
; Maybe you should just use Mem2Inh and Inh2Mem instead (or even convert
; everything into inhlinks to see what would happen!)
; Eval2Implication should be a special case but would only work for
; one-argument versions of the predicates...
; Eval2Member + Member2Inheritance + Rules + Inheritance2Member


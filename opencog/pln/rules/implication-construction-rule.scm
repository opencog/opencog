;; =====================================================================
;; Implication construction rule
;; 
;; P
;; Q
;; |-
;; ImplicationLink
;;    P
;;    Q
;;
;; To properly deal with this we should support deep type and infer
;; that P and Q are over the same domain.
;;
;; Overlaps:
;;
;; - can probably be replaced by a deduction rule with B == Universe
;;
;; ----------------------------------------------------------------------

(define implication-construction-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$P")
        (TypeChoice
           (TypeNode "PredicateNode")
           (TypeNode "LambdaLink")))
     (TypedVariableLink
        (VariableNode "$Q")
        (TypeChoice
           (TypeNode "PredicateNode")
           (TypeNode "LambdaLink")))))

(define implication-construction-body
  (AndLink
     (VariableNode "$P")
     (VariableNode "$Q")
     (NotLink
        (EqualLink
           (VariableNode "$P")
           (VariableNode "$Q")))))

(define implication-construction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-construction-formula")
     (ListLink
        (VariableNode "$P")
        (VariableNode "$Q"))))

(define implication-construction-rule
  (BindLink
     implication-construction-variables
     implication-construction-body
     implication-construction-rewrite))

(define (implication-construction-formula P Q)
  (let* (
         (P-s (cog-stv-strength P))
         (P-c (cog-stv-confidence P))
         (Q-s (cog-stv-strength Q))
         (Q-c (cog-stv-confidence Q))
         ; Compute the implication TV
         (Impl-s Q-s)
         (Impl-c (if (< 0.9 (* Q-s Q-c)) ; Hack to overcome the lack
                                         ; of distributional TV
                        Q-c
                        (* P-c Q-c)))) ; Big hack because the naive
                                       ; formula sucks
    (if (= Impl-c 0) ; Try to avoid constructing informationless
                     ; knowledge
        (cog-undefined-handle)
        (cog-merge-hi-conf-tv!
         (ImplicationLink
            P
            Q)
         (cog-new-stv Impl-s Impl-c)))))

;; Name the rule
(define implication-construction-rule-name
  (DefinedSchemaNode "implication-construction-rule"))
(DefineLink implication-construction-rule-name
  implication-construction-rule)

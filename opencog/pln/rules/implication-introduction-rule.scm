;; =====================================================================
;; Implication introduction rule
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

(define implication-introduction-vardecl
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

(define implication-introduction-pattern
  (AndLink
     (VariableNode "$P")
     (VariableNode "$Q")
     (EvaluationLink
        (GroundedPredicateNode "scm: implication-introduction-precondition")
        (ListLink
           (VariableNode "$P")
           (VariableNode "$Q")))
     (NotLink
        (EqualLink
           (VariableNode "$P")
           (VariableNode "$Q")))))

(define implication-introduction-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-introduction-formula")
     (ListLink
        (ImplicationLink
           (VariableNode "$P")
           (VariableNode "$Q"))
        (VariableNode "$P")
        (VariableNode "$Q"))))

(define implication-introduction-rule
  (BindLink
     implication-introduction-vardecl
     implication-introduction-pattern
     implication-introduction-rewrite))

(define (implication-introduction-precondition P Q)
  (bool->tv (tv-non-null-conf? (implication-introduction-stv-formula P Q))))

(define (implication-introduction-stv-formula P Q)
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
    (stv Impl-s Impl-c)))

(define (implication-introduction-formula Impl P Q)
  (let ((Impl-tv (implication-introduction-stv-formula P Q)))
    (if (tv-non-null-conf? Impl-tv) ; Try to avoid constructing informationless
                                    ; knowledge
        (cog-merge-hi-conf-tv! Impl Impl-tv))))

;; Name the rule
(define implication-introduction-rule-name
  (DefinedSchemaNode "implication-introduction-rule"))
(DefineLink implication-introduction-rule-name
  implication-introduction-rule)

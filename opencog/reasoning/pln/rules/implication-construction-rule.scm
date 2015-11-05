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
;;----------------------------------------------------------------------

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
         (P-tv-s (cog-stv-strength P))
         (P-tv-c (cog-stv-confidence P))
         (Q-tv-s (cog-stv-strength Q))
         (Q-tv-c (cog-stv-confidence Q))
         ; Compute the implication TV
         (Impl-tv-s Q-tv-s)
         (Impl-tv-c (if (< (* Q-tv-s Q-tv-c) 0.9) ; Hack to overcome
                                                  ; the lack of
                                                  ; distributional TV
                        Q-tv-c
                        P-tv-c)))
    (cog-logger-info "There?")
    (cog-set-tv!
     (ImplicationLink
        P
        Q)
     (cog-new-stv Impl-tv-s Impl-tv-c))))

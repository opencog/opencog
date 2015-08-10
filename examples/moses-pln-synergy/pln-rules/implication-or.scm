; =====================================================================
; ImplicationOrRule
;
; ImplicationLink <TV1>
;    A
;    C
; ImplicationLink <TV2>
;    B
;    C
; |-
; ImplicationLink <TV>
;    OrLink
;       A
;       B
;    C
;----------------------------------------------------------------------


(define pln-rule-implication-or
  (BindLink
     (VariableList
        (VariableNode "$A")
        (VariableNode "$B")
        (VariableNode "$C")
        (TypedVariableLink
           (VariableNode "$A")
           (TypeNode "PredicateNode"))
        (TypedVariableLink
           (VariableNode "$B")
           (TypeNode "PredicateNode"))
        (TypedVariableLink
           (VariableNode "$C")
           (TypeNode "PredicateNode"))
     )
     (AndLink
        (ImplicationLink
           (VariableNode "$A")
           (VariableNode "$C")
        )
        (ImplicationLink
           (VariableNode "$B")
           (VariableNode "$C")
        )
        (NotLink (EqualLink (VariableNode "$A") (VariableNode "$B")))
        ;; (NotLink (ImplicationLink (VariableNode "$A") (VariableNode "$B")))
        ;; (NotLink (ImplicationLink (VariableNode "$B") (VariableNode "$A")))
     )
     (ExecutionOutputLink
        (GroundedSchemaNode "scm: pln-formula-implication-or")
        (ListLink
           (ImplicationLink
              (OrLink
                 (VariableNode "$A")
                 (VariableNode "$B")
              )
              (VariableNode "$C")
           )
           (ImplicationLink
              (VariableNode "$A")
              (VariableNode "$C")
           )
           (ImplicationLink
              (VariableNode "$B")
              (VariableNode "$C")
           )
           (VariableNode "$A")
           (VariableNode "$B")
           (VariableNode "$C")
        )
     )
  )
)

(define (pln-formula-implication-or ABC AC BC A B C)
  (cog-set-tv! ABC
   (pln-formula-implication-or-side-effect-free AC BC A B C))
)

;; Computing the strength is based on
;;
;; P(C|A) = P(C,A)/P(A)
;; P(C|B) = P(C,B)/P(B)
;;
;; P(A U B) = P(A)+P(B)-P(A)*P(B)
;;
;; P(C|A U B) = P(C,A U B)/P(A U B)
;; = (P(C,A)+P(C,B)-P(C,A,B))/P(A U B)
;;
;; Let's assume that P(C,A,B) = P(C,A)*P(C,B), then we can write
;;
;; P(C|A U B) = (P(C|A)*P(A) + P(C|B)*P(B) - P(C|A)*P(A)*P(C|B)*P(B)
;;            / (P(A) + P(B) - P(A)*P(B))
;;
(define (pln-formula-implication-or-side-effect-free AC BC A B C)
  (let* 
      ((sAC (cog-stv-strength AC))
       (sBC (cog-stv-strength BC))
       (sA (cog-stv-strength A))
       (sB (cog-stv-strength B))
       (sC (cog-stv-strength C))
       (cAC (cog-stv-confidence AC))
       (cBC (cog-stv-confidence BC))
       (CinterA (* sAC sA))
       (CinterB (* sBC sB)))
    (stv (/ (+ CinterA CinterB (* CinterA CinterB sA sB -1))
            (+ sA sB (* sA sB -1)))
         (min cAC cBC))))

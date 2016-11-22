;; =====================================================================
;; ImplicationImplicantDisjunctionRule
;;
;; ImplicationLink <TV1>
;;    A
;;    C
;; ImplicationLink <TV2>
;;    B
;;    C
;; |-
;; ImplicationLink <TV>
;;    OrLink
;;       A
;;       B
;;    C
;;----------------------------------------------------------------------

(define implication-implicant-disjunction-rule
  (BindLink
     (VariableList
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
        (GroundedSchemaNode "scm: implication-implicant-disjunction-formula")
        (ListLink
           (ImplicationLink
              (VariableNode "$A")
              (VariableNode "$C")
           )
           (ImplicationLink
              (VariableNode "$B")
              (VariableNode "$C")
           )
           (ImplicationLink
              (OrLink
                 (VariableNode "$A")
                 (VariableNode "$B")
              )
              (VariableNode "$C"))))))

(define (implication-implicant-disjunction-formula AC BC ABC)
  (cog-set-tv! ABC
   (implication-implicant-disjunction-side-effect-free-formula AC BC))
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
;; TODO: this is wrong, the assumption should be P(A,B|C) = P(A|C)*P(B|C)
;; Let's assume that P(C,A,B) = P(C,A)*P(C,B), then we can write
;;
;; P(C|A U B) = (P(C|A)*P(A) + P(C|B)*P(B) - P(C|A)*P(A)*P(C|B)*P(B)
;;            / (P(A) + P(B) - P(A)*P(B))
;;
(define (implication-implicant-disjunction-side-effect-free-formula AC BC)
  (let* 
      (
       (A (gar AC))
       (B (gar BC))
       (C (gdr AC))
       (sAC (cog-stv-strength AC))
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

; Name the rule
(define implication-implicant-disjunction-rule-name
  (DefinedSchemaNode "implication-implicant-disjunction-rule"))
(DefineLink implication-implicant-disjunction-rule-name
  implication-implicant-disjunction-rule)

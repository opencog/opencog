;; Consequent Disjunction Elimination Rule. Eliminate one on the
;; argument under a disjunction that is the consequent of an
;; implication.
;;
;; <implication-link>
;;   A
;;   Or
;;     B
;;     C
;; <implication-link>
;;   A
;;   C
;; |-
;; <implication-link>
;;   A
;;   B
;;
;; <implication-link> can be an ImplicationLink, an InheritanceLink,
;; or any variations, eventually scoped as well. Since Or is
;; commutative the rule automatically takes care of eliminating B
;; instead of C as well.
;;
;; TODO for now it is assumed that B and C are conditionally
;; independent on A, this assumption should be replaced by an extra
;; premise (<implication-link> A (And B C)) to find out P(B,C|A).

(define (gen-consequent-disjunction-elimination-rule impl-type var-type)
  (let* ((A (Variable "$A"))
         (B (Variable "$B"))
         (C (Variable "$C")))
    (Bind
      (VariableList
        (TypedVariable A var-type)
        (TypedVariable B var-type)
        (TypedVariable C var-type))
      (impl-type
        A
        (Or
          B
          C))
      (ExecutionOutput
        (GroundedSchema "scm: consequent-disjunction-elimination-formula")
        (List
          ;; Conclusion
          (impl-type
            A
            B)
          ;; Premises
          (impl-type
            A
            (Or
              B
              C))
          (impl-type
            A
            C))))))

(define consequent-disjunction-elimination-inheritance-rule
  (let ((var-type (TypeChoice
                    (TypeNode "ConceptNode")
                    (TypeNode "AndLink")
                    (TypeNode "OrLink")
                    (TypeNode "NotLink"))))
    (gen-consequent-disjunction-elimination-rule InheritanceLink var-type)))

(define consequent-disjunction-elimination-implication-rule
  (let ((var-type (TypeChoice
                    (TypeNode "PredicateNode")
                    (TypeNode "LambdaLink")
                    (TypeNode "AndLink")
                    (TypeNode "OrLink")
                    (TypeNode "NotLink"))))
    (gen-consequent-disjunction-elimination-rule ImplicationLink var-type)))

;; Express P(B|A) given the probabilities of the premises.
;;
;; P(B Union C|A) = P(B|A) + P(C|A) - P(B,C|A)
;; P(B|A) = P(B Union C|A) + P(B,C|A) - P(C|A)
;;
;; Assuming that B and C are independent knowing A, it can be simplified into
;;
;; P(B|A) = P(B Union C|A) + P(B|A)*P(C|A) - P(C|A)
;;
;; 1 = P(B Union C|A)/P(B|A) + P(C|A) - P(C|A)/P(B|A)
;; 1 - P(C|A) = P(B Union C|A)/P(B|A) - P(C|A)/P(B|A)
;; 1 - P(C|A) = (P(B Union C|A) - P(C|A)) / P(B|A)
;; 1 - P(C|A) = (P(B Union C|A) - P(C|A)) / P(B|A)
;; P(B|A) * (1 - P(C|A)) = (P(B Union C|A) - P(C|A))
;; P(B|A) = (P(B Union C|A) - P(C|A)) / (1 - P(C|A))
;;
;; So given that sAB = P(B|A), sABC = P(B Union C|A), sAC = P(C|A) the
;; formula for the strength is
;;
;; if sAC < 1
;;   sAB = (sABC - sAC) / (1 - sAC)
;;
;; If sAC = 1, then we cannot conclude anything about sAB.
;;
;; Also the following precondition should be satisfied
;;
;; P(C|A) <= P(B Union C|A)
(define (consequent-disjunction-elimination-formula conclusion . premises)
  (if (= (length premises) 2)
      (let* ((ABC (list-ref premises 0))
             (AC (list-ref premises 1))
             (sABC (cog-stv-strength ABC))
             (cABC (cog-stv-confidence ABC))
             (sAC (cog-stv-strength AC))
             (cAC (cog-stv-confidence AC))
             (alpha 0.9)                ; Confidence-loss
                                        ; coefficient. TODO replace by
                                        ; something more meaningful
             (AB conclusion)
             (precondition (and (<= sAC sABC) (< sAC 1)))
             (sAB (if precondition
                      (/ (- sABC sAC) (- 1 sAC))
                      1))
             (cAB (if precondition
                      (* alpha (min cABC cAC))
                      0)))
        (if (< 0 cAB)
            (cog-merge-hi-conf-tv! AB (stv sAB cAB))))))

;; Name the rules
(define consequent-disjunction-elimination-inheritance-rule-name
  (DefinedSchemaNode "consequent-disjunction-elimination-inheritance-rule"))
(DefineLink consequent-disjunction-elimination-inheritance-rule-name
  consequent-disjunction-elimination-inheritance-rule)

(define consequent-disjunction-elimination-implication-rule-name
  (DefinedSchemaNode "consequent-disjunction-elimination-implication-rule"))
(DefineLink consequent-disjunction-elimination-implication-rule-name
  consequent-disjunction-elimination-implication-rule)

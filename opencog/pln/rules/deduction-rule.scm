;; =============================================================================
;; DeductionRule
;;
;; <LinkType>
;;   A
;;   B
;; <LinkType>
;;   B
;;   C
;; |-
;; <LinkType>
;;   A
;;   C
;;
;; Due to type system limitations, the rule has been divided into 3:
;;       deduction-inheritance-rule
;;       deduction-implication-rule
;;       deduction-subset-rule
;;
;; -----------------------------------------------------------------------------

(use-modules (opencog logger))

(load "formulas.scm")

;; Generate the corresponding deduction rule given its link-type and
;; the type for each variable (the same for all 3).
(define (gen-deduction-rule link-type var-type)
  (let* ((A (Variable "$A"))
         (B (Variable "$B"))
         (C (Variable "$C"))
         (AB (link-type A B))
         (BC (link-type B C))
         (AC (link-type A C)))
    (Bind
      (VariableList
        (TypedVariable A var-type)
        (TypedVariable B var-type)
        (TypedVariable C var-type))
      (And
        AB
        BC
        (Not (Identical A C)))
      (ExecutionOutput
        (GroundedSchema "scm: deduction-formula")
          (List
            ;; Conclusion
            AC
            ;; Premises
            ;;
            ;; TODO: perhaps A, B, C should be added as premises as
            ;; they are used in the formula.
            AB
            BC)))))

(define deduction-inheritance-rule
  (let ((var-type (TypeChoice
                    (TypeNode "ConceptNode")
                    (TypeNode "AndLink")
                    (TypeNode "OrLink")
                    (TypeNode "NotLink"))))
    (gen-deduction-rule InheritanceLink var-type)))

(define deduction-implication-rule
  (let ((var-type (TypeChoice
                    (TypeNode "PredicateNode")
                    (TypeNode "LambdaLink")
                    (TypeNode "AndLink")
                    (TypeNode "OrLink")
                    (TypeNode "NotLink"))))
    (gen-deduction-rule ImplicationLink var-type)))

(define deduction-subset-rule
  (let ((var-type (TypeChoice
                    (TypeNode "ConceptNode")
                    (TypeNode "AndLink")
                    (TypeNode "OrLink")
                    (TypeNode "NotLink"))))
    (gen-deduction-rule SubsetLink var-type)))

(define (deduction-formula conclusion . premises)
  (if (= (length premises) 2)
    (let*
        ((AC conclusion)
         (AB (list-ref premises 0))
         (BC (list-ref premises 1))
         (sA (cog-stv-strength (gar AB)))
         (cA (cog-stv-confidence (gar AB)))
         (sB (cog-stv-strength (gar BC)))
         (cB (cog-stv-confidence (gar BC)))
         (sC (cog-stv-strength (gdr BC)))
         (cC (cog-stv-confidence (gdr BC)))
         (sAB (cog-stv-strength AB))
         (cAB (cog-stv-confidence AB))
         (sBC (cog-stv-strength BC))
         (cBC (cog-stv-confidence BC))
         (alpha 0.9) ; how much confidence is lost at each deduction step

         ;; Hacks to overcome the lack of distributional TV. If s=1
         ;; and c=0, then assign s to the mode value satisfying the
         ;; deduction consistency constraint (what a pain, let's use
         ;; 0.25 for now).
         (sA (if (and (< 0.99 sA) (<= cA 0)) 0.25 sA))
         (sB (if (and (< 0.99 sB) (<= cB 0)) 0.25 sB))
         (sC (if (and (< 0.99 sC) (<= cC 0)) 0.25 sC)))
      (if (and
           (or (= 0 cA) (= 0 cB) (= 0 cAB)
               (conditional-probability-consistency sA sB sAB))
           (or (= 0 cB) (= 0 cC) (= 0 cBC)
               (conditional-probability-consistency sB sC sBC)))
          (if (< 0.99 (* sB cB))
              ;; Hack to overcome for the lack of distributional
              ;; TV. This covers the case where B fully confidently
              ;; tends to 1. See formulas.scm Simple Deduction
              ;; Formula comment for more explanations. This
              ;; overlaps with the implication-introduction-rule.
              (let ((sAC sC)
                    (cAC (* alpha cA cC)))
                (if (and (< 1e-8 sAC) (< 1e-8 cAC)) ;; Don't create zero
                                              ;; knowledge. Note that
                                              ;; sAC == 0 is not zero
                                              ;; knowledge but it's
                                              ;; annoying in the
                                              ;; current hacky
                                              ;; situation.
                    (cog-merge-hi-conf-tv! AC (stv sAC cAC))
                    (cog-undefined-handle)))
              ;; Branch if sB * cB <= 0.99
              (let* ((sAC (if (or (< 0.99 (* sAB cAB)) (< 0.99 (* sBC cBC)))
                              ;; Hack to overcome for the lack of
                              ;; distributional TV. This covers the case
                              ;; where little is known about A and B
                              ;; (i.e. their strength is meaningless), yet
                              ;; we can confidently calculate sAC because
                              ;; sAB and sBC are so high anyway.
                              (* sAB sBC)
                              ;; Otherwise fall back on the naive formula
                              (simple-deduction-strength-formula sA sB sC sAB sBC)))
                     (cAC (min cAB cBC))
                     ;; Unless the 2 implication are fully confident
                     ;; decrease the confidence by some factor. I'm not
                     ;; sure how justify this for now, it's perhaps a
                     ;; bad hack.
                     (cAC (* (if (< cAC 0.99) alpha 1.0) cAC)))
                (if (and (< 1e-8 sAC) (< 1e-8 cAC)) ;; Don't create zero
                                              ;; knowledge. Note that
                                              ;; sAC == 0 is not zero
                                              ;; knowledge but it's
                                              ;; annoying in the
                                              ;; current hacky
                                              ;; situation.
                    (cog-merge-hi-conf-tv! AC (stv sAC cAC))
                    (cog-undefined-handle))))
          (cog-undefined-handle)))))

;; Name the rules
(define deduction-inheritance-rule-name
  (DefinedSchemaNode "deduction-inheritance-rule"))
(DefineLink deduction-inheritance-rule-name
  deduction-inheritance-rule)

(define deduction-implication-rule-name
  (DefinedSchemaNode "deduction-implication-rule"))
(DefineLink deduction-implication-rule-name
  deduction-implication-rule)

(define deduction-subset-rule-name
  (DefinedSchemaNode "deduction-subset-rule"))
(DefineLink deduction-subset-rule-name
  deduction-subset-rule)

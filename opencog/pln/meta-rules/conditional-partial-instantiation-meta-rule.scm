;; =======================================================================
;; Conditional Partial Instantiation Meta Rule
;;
;; ImplicationScopeLink
;;   V1, V2, V3
;;   P
;;   Q
;; |-
;;   T2
;;   T3
;;   |-
;;   ImplicationScopeLink
;;     V1
;;     P[V2->T2,V3->T3]
;;     Q[V2->T2,V3->T3]
;;
;; The fact that there 3 variables is hardcoded for now.
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Conditional partial instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;; Hard code a 3-ary ImplicationScope turning into a unary
;; ImplicationScope where V2 and V3 are substituted by terms.
(define conditional-partial-instantiation-meta-rule
  (let* ((V1 (Variable "$V1"))
         (V2 (Variable "$V2"))
         (V3 (Variable "$V3"))
         (V1Type (Variable "$V1Type"))
         (V2Type (Variable "$V2Type"))
         (V3Type (Variable "$V3Type"))
         (VariableT (Type "VariableNode"))
         (TypeChoiceT (Type "TypeChoice"))
         (TypeT (Type "TypeNode"))
         (VariableTypeT (TypeChoice TypeChoiceT TypeT))
         (TypedV1 (TypedVariable V1 VariableT))
         (TypedV2 (TypedVariable V2 VariableT))
         (TypedV3 (TypedVariable V3 VariableT))
         (TypedV1Type (TypedVariable V1Type VariableTypeT))
         (TypedV2Type (TypedVariable V2Type VariableTypeT))
         (TypedV3Type (TypedVariable V3Type VariableTypeT))
         (P (Variable "$P"))
         (Q (Variable "$Q"))
         ;; Meta rule variable declaration
         (meta-vardecl (VariableList
                         TypedV1 TypedV2 TypedV3
                         TypedV1Type TypedV2Type TypedV3Type
                         P Q))
         ;; Meta rule main clause
         (implication (LocalQuote
                        (ImplicationScope
                          (VariableList
                            (TypedVariable V1 V1Type)
                            (TypedVariable V2 V2Type)
                            (TypedVariable V3 V3Type))
                          P
                          Q)))
         ;; Meta rule precondition
         (meta-precondition (Evaluation
                              (GroundedPredicate "scm: gt-zero-confidence")
                              implication))
         ;; Meta rule pattern
         (meta-pattern (And implication meta-precondition))
         ;; Produced rule variable declaration. V2 and V3 are to be
         ;; substituted.
         (produced-vardecl (VariableList
                             (TypedVariable V2 V2Type)
                             (TypedVariable V3 V3Type)))
         ;; Produced rule pattern. Just look for groudings of V2 and V3
         (produced-pattern (And V2 V3))
         ;; Produced rule rewrite. Apply formula to calculate the TV
         ;; over the partially substituted ImplicationScope.
         (produced-rewrite (ExecutionOutput
                            (GroundedSchema "scm: conditional-partial-instantiation-formula")
                            (Unquote
                              (List
                                ;; Conclusion
                                (LocalQuote
                                  (ImplicationScope
                                    (TypedVariable V1 V1Type) ; only V1 remains
                                    P
                                    Q))
                                ;; Premise
                                implication))))
         ;; Meta rule rewrite
         (meta-rewrite (Quote (Bind
                          (Unquote produced-vardecl)
                          (Unquote produced-pattern)
                          produced-rewrite ; the Unquote appears
                                           ; inside it, to avoid
                                           ; running the
                                           ; ExecutionOutput
                          ))))
    (Bind
      meta-vardecl
      meta-pattern
      meta-rewrite)))

;; Return TrueTV iff A's confidence is greater than 0
(define (gt-zero-confidence A)
  (bool->tv (> (cog-stv-confidence A) 0)))

(define (conditional-partial-instantiation-formula PImpl Impl)
  ;; For now merely put the TV of Impl on PImpl
  (cog-set-tv! PImpl (cog-tv Impl)))

;; Name the meta rule
(define conditional-partial-instantiation-meta-rule-name
  (DefinedSchemaNode "conditional-partial-instantiation-meta-rule"))
(DefineLink conditional-partial-instantiation-meta-rule-name
  conditional-partial-instantiation-meta-rule)

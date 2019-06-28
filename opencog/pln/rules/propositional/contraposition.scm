;; =============================================================================
;; Contraposition rule
;;
;; <implication-type-link>
;;   A
;;   B
;; |-
;; <implication-type-link>
;;   Not B
;;   Not A
;;
;; where <implication-type-link> can be
;;
;; ImplicationLink
;; InheritanceLink
;; ImplicationScopeLink
;; etc
;;
;; TODO: maybe we want shortcut rules such as
;;
;; <implication-type-link>
;;   Not A
;;   B
;; |-
;; <implication-type-link>
;;   Not B
;;   A
;;
;; to avoid having to introduce negations in order to match the rule
;; conclusion, as the backward chainer would.
;; -----------------------------------------------------------------------------

;; Generate the corresponding contraposition rule given its unscope
;; link-type.
(define (gen-contraposition-rule link-type)
  (let* ((A (Variable "$A"))
         (B (Variable "$B"))
         (AB (link-type A B))
         (NBNA (link-type (Not B) (Not A)))
         (vardecl (VariableList A B))
         (clause AB)
         (precondition (Evaluation
                         (GroundedPredicate "scm: gt-zero-confidence")
                         AB))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: contraposition-formula")
                    (List
                      NBNA
                      AB
                      A
                      B))))
    (Bind
      vardecl
      (And
        (Present clause)
        precondition)
      rewrite)))

;; Generate the corresponding contraposition rule given its scope
;; link-type. For now it is crisp, because having to generate the
;; premises (Lambda X P) and (Lambda X Q) is a bit too
;; heavy. Ultimately we probably want a crip and non crisp
;; version. Crisp here means that the rule will only be triggered if
;; the TV over implication is (stv 1 1), that way, although it is
;; crisp, it is not inconsistent with PLN.
(define (gen-crisp-contraposition-scope-rule scope-link-type)
  (let* ((V (Variable "$V"))
         (P (Variable "$P"))
         (Q (Variable "$Q"))
         (PQ (Quote (scope-link-type
               (Unquote V)
               (Unquote P)
               (Unquote Q))))
         (NQNP (Quote (scope-link-type
                 (Unquote V)
                 (Unquote (Not Q))
                 (Unquote (Not P)))))
         (vardecl (VariableList
                    (TypedVariable V (TypeChoice
                                       (TypeNode "VariableNode")
                                       (TypeNode "VariableList")
                                       (TypeNode "TypedVariableLink")))
                    P Q))
         (clause PQ)
         (precondition (Evaluation
                         (GroundedPredicate "scm: crisp-contraposition-scope-precondition")
                         PQ))
         (rewrite (ExecutionOutput
                    (GroundedSchema "scm: crisp-contraposition-scope-formula")
                    (List
                      NQNP
                      PQ))))
    (Bind
      vardecl
      (And
        clause
        precondition)
      rewrite)))

(define (crisp-contraposition-scope-precondition PQ)
  (bool->tv (and (< 0.999 (cog-mean PQ)) (< 0.999 (cog-confidence PQ)))))

;; Given sAB = P(B|A), sA = P(A), sB = P(B), let's calculate
;; sNBNA = P(Not A | Not B)
;;       = P(Not A inter Not B) / P(Not B)
;;       = P(Not(A union B)) / P(Not B)
;;       = (1 - P(A union B)) / (1 - P(B))
;;       = (1 - P(A) - P(B) + P(A inter B)) / (1 - P(B))
;;       = (1 - P(A) - P(B) + P(B|A)*P(A)) / (1 - P(B))
;;       = (1 - sA - sB + sAB*sA) / (1 - sB)
;;
;; One may notice that if sAB = 1, the crisp case, then the formula
;; can be simplified into
;;
;; sNBNA = (1 - sA - sB + sA) / (1 - sB)
;;       = 1
(define (contraposition-formula conclusion . premises)
  (if (= (length premises) 3)
    (let*
        ((NBNA conclusion)
         (AB (list-ref premises 0))
         (A (list-ref premises 1))
         (B (list-ref premises 2))
         (sAB (cog-mean AB))
         (cAB (cog-confidence AB))
         (sA (cog-mean A))
         (cA (cog-confidence A))
         (sB (cog-mean B))
         (cB (cog-confidence B)))
      (if (and (< 0.999 sAB) (< 0.999 cAB))
          (cog-merge-hi-conf-tv! NBNA (stv sAB cAB))
          (if (> 1 sB)
              (let* ((sNBNA ((+ 1 (- sA) (- sB) (* sAB sA)) / (+ 1 (- sB))))
                     (cNBNA (min cAB cA cB)))
                (cog-merge-hi-conf-tv! NBNA (stv sNBNA cNBNA))))))))

(define (crisp-contraposition-scope-formula conclusion . premises)
  (if (= (length premises) 1)
    (let*
        ((NQNP conclusion)
         (PQ (list-ref premises 0))
         (sPQ (cog-mean PQ))
         (cPQ (cog-confidence PQ)))
      (if (and (< 0.999 sPQ) (< 0.999 cPQ))
          (cog-merge-hi-conf-tv! NQNP (stv sPQ cPQ))))))

;; Gen and name the rules
(define crisp-contraposition-implication-scope-rule
  (gen-crisp-contraposition-scope-rule ImplicationScopeLink))
(define crisp-contraposition-implication-scope-rule-name
  (DefinedSchemaNode "crisp-contraposition-implication-scope-rule"))
(DefineLink crisp-contraposition-implication-scope-rule-name
  crisp-contraposition-implication-scope-rule)

(define contraposition-implication-rule
  (gen-contraposition-rule ImplicationLink))
(define contraposition-implication-rule-name
  (DefinedSchemaNode "contraposition-implication-rule"))
(DefineLink contraposition-implication-rule-name
  contraposition-implication-rule)

(define contraposition-inheritance-rule
  (gen-contraposition-rule InheritanceLink))
(define contraposition-inheritance-rule-name
  (DefinedSchemaNode "contraposition-inheritance-rule"))
(DefineLink contraposition-inheritance-rule-name
  contraposition-inheritance-rule)

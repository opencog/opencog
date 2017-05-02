;; =======================================================================
;; Conditional Full Instantiation Meta Rule
;;
;; Multiple forms are implemented
;;
;; 1. ImplicationScopeLink
;;
;; ImplicationScopeLink
;;   V
;;   P
;;   Q
;; |-
;;   T
;;   P[V->T]
;;   |-
;;   Q[V->T]
;;
;; 2. InheritanceLink
;;
;; InheritanceLink
;;   A
;;   B
;; |-
;;   T
;;   MemberLink
;;     T
;;     A
;;   |-
;;   MemberLink
;;     T
;;     B
;;
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Conditional full instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define conditional-full-instantiation-implication-scope-meta-rule
  (let* ((V (Variable "$V"))
         (VariableT (Type "VariableNode"))
         (VariableListT (Type "VariableList"))
         (TypedVariableT (Type "TypedVariableLink"))
         (VardeclT (TypeChoice VariableT VariableListT TypedVariableT))
         (P (Variable "$P"))
         (Q (Variable "$Q"))

         ;; Meta rule variable declaration
         (meta-vardecl (VariableList
                         (TypedVariable V VardeclT)
                         P Q))
         ;; Meta rule main clause
         (implication (Quote
                        (ImplicationScope
                          (Unquote V)
                          (Unquote P)
                          (Unquote Q))))
         ;; Meta rule precondition
         (meta-precondition (Evaluation
                              (GroundedPredicate "scm: gt-zero-confidence")
                              implication))
         ;; Meta rule pattern
         (meta-pattern (And implication meta-precondition))

         ;; Produced rule variable declaration. P and Q will now be
         ;; content rather than variables.
         (produced-vardecl V)
         ;; Produced rule precondition. P must have a positive confidence
         (produced-precondition (Evaluation
                                  (GroundedPredicate "scm: gt-zero-confidence")
                                  P))
         ;; Produced rule pattern. Look for groundings of P that meet
         ;; the precondition.
         (produced-pattern (And P produced-precondition))
         ;; Produced rule rewrite. Apply formula to calculate the TV
         ;; over Q.
         (produced-rewrite (ExecutionOutput
                            (GroundedSchema "scm: conditional-full-instantiation-scope-formula")
                            (Unquote
                              (List
                                ;; Conclusion
                                Q
                                ;; Premises. Both P and the
                                ;; implication are required to
                                ;; calculate the TV over Q.
                                P
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

;; (define conditional-full-instantiation-inheritance-meta-rule
;;   (let* ((X (Variable "$X"))
;;          (A (Variable "$A"))
;;          (B (Variable "$B"))
;;          (ConceptT (Type "ConceptNode"))
;;          ;; Meta rule variable declaration
;;          (meta-vardecl (VariableList
;;                          (TypedVariable A ConceptT)
;;                          (TypedVariable B ConceptT)))
;;          ;; Meta rule main clause
;;          (inheritance (Inheritance A B))
;;          ;; Meta rule precondition
;;          (meta-precondition (Evaluation
;;                               (GroundedPredicate "scm: gt-zero-confidence")
;;                               inheritance))
;;          ;; Meta rule pattern
;;          (meta-pattern (And inheritance meta-precondition))

;;          ;; Produced rule variable declaration. A and B will now be
;;          ;; content rather than variables.
;;          (produced-vardecl X)
;;          (produced-clause (Member X A))
;;          ;; Produced rule preconditions. A and A(X) must have a positive confidence
;;          (produced-precondition-1 (Evaluation
;;                                     (GroundedPredicate "scm: gt-zero-confidence")
;;                                     A))
;;          (produced-precondition-2 (Evaluation
;;                                     (GroundedPredicate "scm: gt-zero-confidence")
;;                                     (Member X A)))
;;          ;; Produced rule pattern. Look for groundings of P that meet
;;          ;; the precondition.
;;          (produced-pattern (And
;;                              produced-clause
;;                              produced-precondition-1
;;                              produced-precondition-2))
;;          ;; Produced rule rewrite. Apply formula to calculate the TV
;;          ;; over B(X).
;;          (produced-rewrite (ExecutionOutput
;;                             (GroundedSchema "scm: conditional-full-instantiation-formula")
;;                             (Unquote
;;                               (List
;;                                 ;; Conclusion
;;                                 (Member X B)
;;                                 ;; Premises. Both P and the
;;                                 ;; implication are required to
;;                                 ;; calculate the TV over Q.
;;                                 (Member X A)
;;                                 inheritance))))
;;          ;; Meta rule rewrite
;;          (meta-rewrite (Quote (Bind
;;                           (Unquote produced-vardecl)
;;                           (Unquote produced-pattern)
;;                           produced-rewrite ; the Unquote appears
;;                                            ; inside it, to avoid
;;                                            ; running the
;;                                            ; ExecutionOutput
;;                           ))))
;;     (Bind
;;       meta-vardecl
;;       meta-pattern
;;       meta-rewrite)))

;; The TV of Q is calculated as follows
;;
;;    Strength:
;;
;;       Let s be the strength of the ImplicationScope. We define the
;;       formula as follows
;;
;;       Q(a) = s*P(a)
;;
;;       where P(a) and Q(a) are respectively the membership strengths
;;       of a in P and Q.
;;
;;       Proof: s, the implication strength is, by definition (see PLN
;;       book, Section 2.4.1.1, page 29)
;;
;;            Sum_x min(P(x), Q(x))
;;       s = ----------------------
;;                 Sum_x P(x)
;;
;;       where P(x), Q(x) are respectively the membership degrees of x
;;       to P and Q. Let's assume that a is the substitution term, and
;;       extract it from the sums
;;
;;            min(P(a), Q(a) + Sum_{x in X-a} min(P(x), Q(x))
;;       s = ------------------------------------------------
;;                      P(a) + Sum_{x in X-a} P(x)
;;
;;
;;       Let's move the denominator to the left side and distribution s
;;
;;       s * P(a) + s * Sum_{x in X-a} P(x)
;;       = min(P(a), Q(a) + Sum_{x in X-a} min(P(x), Q(x))
;;
;;       Let's divide every side by Sum_{x in X-a} P(x)
;;
;;      s * P(a)               min(P(a), Q(a))     Sum_{x in X-a} min(P(x), Q(x))
;; ------------------- + s = ------------------- + ------------------------------
;; Sum_{x in X-a} P(x)       Sum_{x in X-a} P(x)       Sum_{x in X-a} P(x)
;;
;;       Now, let's assume that P's and Q's supports are infinite, thus
;;
;;       Sum_{x in X-a} min(P(x), Q(x))   Sum_x min(P(x), Q(x))
;;       ------------------------------ = --------------------- = s
;;            Sum_{x in X-a} P(x)              Sum_x P(x)
;;
;;       This allows us to simplify
;;
;;            s * P(a)               min(P(a), Q(a))
;;       ------------------- + s = ------------------- + s
;;       Sum_{x in X-a} P(x)       Sum_{x in X-a} P(x)
;;
;;       and finally
;;
;;       s * P(a) = min(P(a), Q(a))
;;
;;       if s = 1, P(a) = min(P(a), Q(a)) thus Q(a) >= P(a)
;;
;;       if s < 1, s * P(a) = min(P(a), Q(a)) thus Q(a) = s * P(a),
;;       because min(P(a), Q(a)) < P(a), thus Q(a) < P(a), which by
;;       definition of the min, Q(a) = min(P(a), Q(a)).
;;
;;       So Q(a) is only determined if s < 1, Q(a) = s * P(a). However, for
;;       the sake of continuity, we'll assume that Q(a) = P(a) when s = 1.
;;
;;    Confidence:
;;
;;       This one is much trickier. To estimate it we need to know how
;;       well the ImplicationScope explains the observations (i.e. the
;;       variable substitution terms). Basically we need to use Bayes
;;       rule
;;
;;       Pr(M|D) = Pr(D|M) * Pr(M) / Pr(D)
;;
;;       where M is a model (here the ImplicationScope), D the
;;       observations. Beware that this information is NOT captured by
;;       the ImplicationScope TV alone. More research is needed but
;;       the idea would be to query the AtomSpace for the knowledge
;;       corresponding to the Bayes equation above.
;;
;;       For now though the following simple (and rather wrong)
;;       heuristic is used
;;
;;       Q(a).c = c * P(a).c * (1-P.s)
;;
;;       where c is the confidence of the ImplicationScope and P.s is
;;       the strength of P before instantiation.

;; Given various TVs calculate the TV of Q(a)
(define (conditional-full-instantiation-tv-formula Pinst-tv Impl-tv P-tv)
  (let* ((Impl-s (tv-mean Impl-tv))
         (Impl-c (tv-conf Impl-tv))
         (P-s (tv-mean P-tv))
         (P-c (tv-conf P-tv))
         ;; Hacks to overcome the lack of distributional TV. If s=1
         ;; and c=0, then assign s to the mode value satisfying the
         ;; deduction consistency constraint (what a pain, let's use
         ;; 0.25 for now).
         (P-s (if (and (< 0.99 P-s) (<= P-c 0)) 0.25 P-s))
         (Pinst-s (tv-mean Pinst-tv))
         (Pinst-c (tv-conf Pinst-tv))
         (Qinst-s (* Impl-s Pinst-s))
         ;; (Qinst-c (* Impl-c Pinst-c (if (< 0 P-c ) (- 1 P-s) 1))))
         (Qinst-c (* Impl-c Pinst-c (- 1 P-s))))
    (stv Qinst-s Qinst-c)))

(define (conditional-full-instantiation-scope-formula Qinst Pinst Impl)
  (let* ((Impl-outgoings (cog-outgoing-set Impl))
         (P (cadr Impl-outgoings))
         (Pinst-tv (cog-tv Pinst))
         (Impl-tv (cog-tv Impl))
         (P-tv (cog-tv P))
         (Qinst-tv (conditional-full-instantiation-tv-formula Pinst-tv Impl-tv P-tv)))
    (if (< 0 (tv-conf Qinst-tv)) ; avoid creating informationless knowledge
        (cog-merge-hi-conf-tv! Qinst Qinst-tv))))

;; Like conditional-full-instantiation-formula but assumes a non scope
;; link.
(define (conditional-full-instantiation-formula Qinst Pinst Impl)
  (let* ((Impl-outgoings (cog-outgoing-set Impl))
         (P (car Impl-outgoings))
         (Pinst-tv (cog-tv Pinst))
         (Impl-tv (cog-tv Impl))
         (P-tv (cog-tv P))
         (Qinst-tv (conditional-full-instantiation-tv-formula Pinst-tv Impl-tv P-tv)))
    (if (< 0 (tv-conf Qinst-tv)) ; avoid creating informationless knowledge
        (cog-merge-hi-conf-tv! Qinst Qinst-tv))))

;; Name the implication scope meta rule
(define conditional-full-instantiation-implication-scope-meta-rule-name
  (DefinedSchemaNode "conditional-full-instantiation-implication-scope-meta-rule"))
(DefineLink conditional-full-instantiation-implication-scope-meta-rule-name
  conditional-full-instantiation-implication-scope-meta-rule)

;; ;; Name the inheritance meta rule
;; (define conditional-full-instantiation-inheritance-meta-rule-name
;;   (DefinedSchemaNode "conditional-full-instantiation-inheritance-meta-rule"))
;; (DefineLink conditional-full-instantiation-inheritance-meta-rule-name
;;   conditional-full-instantiation-inheritance-meta-rule)

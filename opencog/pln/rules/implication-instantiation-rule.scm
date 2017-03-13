;; =======================================================================
;; Implication Instantiation rule
;; (TODO add wiki page)
;;
;; ImplicationScopeLink
;;    V
;;    P
;;    Q
;; T
;; |-
;; Q[V->T]
;;
;; where V is a variable or a list of variables, P is a condition, Q
;; is the implicand, T is an atom (or a list of atoms) to substitute
;; and Q[V->T] is Q where V has been substituted by T.
;;
;; As currently implemented T is not explicitely in the
;; premises. Instead it is queried directly by the rule's formula.
;; -----------------------------------------------------------------------

(use-modules (srfi srfi-1))

(use-modules (opencog exec))
(use-modules (opencog logger))

(load "instantiation.scm")

;;;;;;;;;;;;;;;;;;;;;;;
;; Helper definition ;;
;;;;;;;;;;;;;;;;;;;;;;;

(define implication-full-instantiation-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeChoice
           (TypeNode "TypedVariableLink")
           (TypeNode "VariableList")))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-instantiation-body
  (QuoteLink (ImplicationScopeLink
     (UnquoteLink (VariableNode "$TyVs"))
     (UnquoteLink (VariableNode "$P"))
     (UnquoteLink (VariableNode "$Q")))))

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Implication full instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define implication-full-instantiation-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-full-instantiation-formula")
     (ListLink
        implication-instantiation-body)))

;; Only try to match an ImplicationScopeLink with a type restricted
;; variable in the ImplicationScopeLink variable definition. The choice of
;; the substitution term is done randomly in
;; implication-full-instantiation-formula. All scoped variables are
;; instantiated.
(define implication-full-instantiation-rule
  (BindLink
     implication-full-instantiation-variables
     implication-instantiation-body
     implication-full-instantiation-rewrite))

;; This function
;;
;; 1. randomly selects a substitution term (or a tuple of substitution
;;    terms, if the ImplicationScopeLink has multiple variables in scope)
;;    that meets the implication's condition (the implicant),
;;
;; 2. performs the substitution.
;;
;; 3. calculates its TV as follows
;;
;;    Strength:
;;
;;       Let s be the strength of the Implication TV. Let Ps be the
;;       strength of the TV of P[V->T], and Qs, the strength of the TV of
;;       Q[V->T] to calculate. We define the formula as follows
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
;;       Now, let's assume that P's and Q's support are infinite, thus
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
;;       Let c be the confidence of the implication TV, P(a).c the
;;       confidence of the TV of P[V->a], and Q(a).c the confidence of
;;       the TV of Q[V->a]. Q(a).c is calculated as follows
;;
;;       Q(a).c = c * P(a).c * (1-P.s)
;;
;;       where P.s is the strength of predicate P, the precondition.
;;
;;       Informal (wrong) proof: The c * P(a).c part makes sense
;;       intuitively. The (1-P.s) is less intuitive, but here is the
;;       argument for it. The larger P the more elements which are not
;;       `a` are used to calculate the probability on the
;;       implication. Anything that isn't `a` is in fact a potential
;;       distraction toward the true membership of Q(a).
;;
;;       EDIT: To really address that you need use Bayes rule
;;
;;       P(M|D) = P(D|M) * P(M) / P(D)
;;
;;       and use that over known models (implications, whatever) to
;;       estimate P(D'|D)
;;
(define (implication-full-instantiation-formula Impl)
  (let* ((Impl-outgoings (cog-outgoing-set Impl))
         (Impl-s (cog-stv-strength Impl))
         (Impl-c (cog-stv-confidence Impl))
         (TyVs (car Impl-outgoings))
         (P (cadr Impl-outgoings))
         (P-s (cog-stv-strength P))
         (P-c (cog-stv-confidence P))
         ;; Hacks to overcome the lack of distributional TV. If s=1
         ;; and c=0, then assign s to the mode value satisfying the
         ;; deduction consistency constraint (what a pain, let's use
         ;; 0.25 for now).
         (P-s (if (and (< 0.99 P-s) (<= P-c 0)) 0.25 P-s))
         (Q (caddr Impl-outgoings))
         (terms (if (= 0 Impl-c) ; don't try to instantiate zero
                                 ; knowledge implication
                    (cog-undefined-handle)
                    (select-conditioned-substitution-terms TyVs P))))
    (if (equal? terms (cog-undefined-handle))
        (cog-undefined-handle)
        ;; Substitute the variables by the terms in P and Q. In P to
        ;; get its TV, in Q cause it's the rule output.
        (let* (
               (Pput (PutLink (LambdaLink TyVs P) terms))
               (Pinst (cog-execute! Pput))
               (Pinst-s (cog-stv-strength Pinst))
               (Pinst-c (cog-stv-confidence Pinst))
               (Qput (PutLink (LambdaLink TyVs Q) terms))
               (Qinst (cog-execute! Qput))
               (Qinst-s (* Impl-s Pinst-s))
               ;; (Qinst-c (* Impl-c Pinst-c (if (< 0 P-c ) (- 1 P-s) 1))))
               (Qinst-c (* Impl-c Pinst-c (- 1 P-s))))
          ;; Remove the PutLinks to not pollute the atomspace
          ;; TODO: replace this by something more sensible
          ;; (extract-hypergraph Pput)
          ;; (extract-hypergraph Qput)
          (if (= 0 Qinst-c) ; avoid creating informationless knowledge
              (cog-undefined-handle)
              (cog-merge-hi-conf-tv! Qinst (stv Qinst-s Qinst-c)))))))

;; Name the rule
(define implication-full-instantiation-rule-name
  (DefinedSchemaNode "implication-full-instantiation-rule"))
(DefineLink implication-full-instantiation-rule-name
  implication-full-instantiation-rule)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Implication partial instantiation rule ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define implication-partial-instantiation-variables
  (VariableList
     (TypedVariableLink
        (VariableNode "$TyVs")
        (TypeNode "VariableList"))
     (VariableNode "$P")
     (VariableNode "$Q")))

(define implication-partial-instantiation-rewrite
  (ExecutionOutputLink
     (GroundedSchemaNode "scm: implication-partial-instantiation-formula")
     (ListLink
        implication-instantiation-body)))

;; Like implication-full-instantiation-rule but only instantiate one
;; variable amonst a variable list (if there is just one variable in
;; the implication scope, then this rule will not be invoked).
(define implication-partial-instantiation-rule
  (BindLink
     implication-partial-instantiation-variables
     implication-instantiation-body
     implication-partial-instantiation-rewrite))

;; This function
;;
;; 1. randomly selects a substitution term that partially meets the
;;    implication's condition (the implicant),
;;
;; 2. performs the substitution,
;;
;; 3. calculates its TV (just the TV on the implication link for now,
;; in principle there might better ways)
;;
;; TODO: To make this function better a form of partial pattern
;; matching should be supported. Probably enabling self grounding in
;; the pattern matcher would do the trick (see
;; PatternMatchEngine::self_compare)
(define (implication-partial-instantiation-formula Impl)
  (let* (
         (Impl-outgoings (cog-outgoing-set Impl))
         (TyVs (car Impl-outgoings))
         (P (cadr Impl-outgoings))
         (Q (caddr Impl-outgoings))
         (TyVs-outgoings (cog-outgoing-set TyVs))
         (TyVs-outgoings-len (length TyVs-outgoings))
                                        ; Select all potential
                                        ; substitution terms
         (terms (select-conditioned-substitution-terms TyVs P))
                                        ; Choose the index of the
                                        ; variable to substitute
         (rnd-index (random TyVs-outgoings-len))
                                        ; Take it's corresponding
                                        ; variable
         (TyV (list-ref TyVs-outgoings rnd-index))
                                        ; Build a VariableList of the
                                        ; remaining variables
         (TyVs-remain-list (rm-list-ref TyVs-outgoings rnd-index))
         (TyVs-remain-len (length TyVs-remain-list))
         (TyVs-remain (apply cog-new-link 'VariableList TyVs-remain-list)))
    (if (equal? terms (cog-undefined-handle))
        terms
        (cog-set-tv!
         (let* (
                                        ; Take the corresponding term
                (term (list-ref (cog-outgoing-set terms) rnd-index))
                                        ; Substitute the variable by
                                        ; the term in the P and Q bodies
                (P-put (PutLink (LambdaLink TyV P) term))
                (Q-put (PutLink (LambdaLink TyV Q) term))
                (P-inst (cog-execute! P-put))
                (Q-inst (cog-execute! Q-put))
                                        ; If there is only one
                                        ; variable left, discard the
                                        ; VariableLink
                (TyVs-remain (if (= TyVs-remain-len 1)
                                 (gar TyVs-remain)
                                 TyVs-remain)))
           ;; Remove the put links to not populate the atomspace
           (extract-hypergraph P-put)
           (extract-hypergraph Q-put)
           (if (> TyVs-remain-len 0)
                                        ; If there are some variables
                                        ; left, rebuild the
                                        ; ImplicationLink with the
                                        ; remaining variables
               (ImplicationScopeLink TyVs-remain P-inst Q-inst)
                                        ; Otherwise just return the
                                        ; Q instance
               Q-inst))
         (cog-tv Impl)))))

;; Name the rule
(define implication-partial-instantiation-rule-name
  (DefinedSchemaNode "implication-partial-instantiation-rule"))
(DefineLink implication-partial-instantiation-rule-name
  implication-partial-instantiation-rule)

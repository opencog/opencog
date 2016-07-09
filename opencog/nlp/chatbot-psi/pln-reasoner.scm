;; Background PLN reasoning
;;
;; Very simplistic and hacky at the moment, loop of 2 rules
;;
;; 1. Preprocessing to turn r2l outputs into something that PLN can reason on
;;
;; 2. Limited induction reasoning
;;
;; The reasoner doesn't use the URE. Instead if merely applies the 2
;; rules on after then other in a loop.

(load "pln-states.scm")

(use-modules (opencog logger))
(use-modules (opencog query))

;; Load PLN rule implication direct evaluation
(load-from-path "opencog/pln/rules/implication-direct-evaluation-rule.scm")

;;;;;;;;;;;;;;;
;; L2S rules ;;
;;;;;;;;;;;;;;;

;; Rule to turn something like
;;
;; (InheritanceLink
;;    (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    (ConceptNode "Ben" (stv 0.029411765 0.0012484394))
;; )
;; (ImplicationLink
;;    (PredicateNode "happy@1d08ff8b-4149-4362-97ef-9103a307a879")
;;    (PredicateNode "happy" (stv 0.25 0.0012484394))
;; )
;; (EvaluationLink
;;    (PredicateNode "happy@1d08ff8b-4149-4362-97ef-9103a307a879")
;;    (ListLink
;;       (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    )
;; )
;; (InheritanceLink
;;    (InterpretationNode "sentence@bf0cc09b-b2c3-4373-9573-6e8bb60a6b5f_parse_0_interpretation_$X")
;;    (DefinedLinguisticConceptNode "DeclarativeSpeechAct")
;; )
;;
;; plus possibly (TBD)
;;
;; (EvaluationLink
;;    (DefinedLinguisticPredicateNode "definite")
;;    (ListLink
;;       (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    )
;; )
;; (InheritanceLink
;;    (SpecificEntityNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    (ConceptNode "Ben" (stv 0.029411765 0.0012484394))
;; )
;;
;; into
;;
;; (EvaluationLink (stv 1 0.1)
;;    (Predicate "happy")
;;    (ListLink
;;       (ConceptNode "Ben")))
;;
;; The 0.1 is largely arbitrary and convey the fact that we only have
;; limited evidences.
;;
;; To do well this rule should be replaced by an axioms relating the
;; cohesiveness of the involved predicate + PLN rules. THe axiom would
;; look like
;;
;; Implication
;;   VariableList
;;     TypedVariable
;;       Variable "$P"
;;       Type "Predicate"
;;     TypedVariable
;;       Variable "$X"
;;       Type "Concept"
;;     TypedVariable
;;       Variable "$Y"
;;       Type "Concept"
;;   And
;;     Evaluation
;;       Predicate "is-cohesive"
;;       Variable "$P"
;;     Evaluation
;;       Variable "$P"
;;       Variable "$X"
;;     Similarity
;;       Variable "$X"
;;       Variable "$Y"
;;   Evaluation
;;     Variable "$P"
;;     Variable "$X"
;;
;; Then PLN would use, as well as estimate the Similarities between
;; the arguments to produce some equivalent (but with better true
;; value estimate) outcome.
;;
;; We call these rule l2s, which stands for logic to
;; semantics. Eventually maybe these can be turned into a rule-base,
;; using the URE, but for now it's more like a hack.

(define unary-predicate-speech-act-l2s-vardecl
   (VariableList
      (TypedVariable
         (Variable "$element-instance")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$element")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$predicate-instance")
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$predicate")
         (Type "PredicateNode"))))

(define unary-predicate-speech-act-l2s-pattern
   (And
      (Inheritance
         (Variable "$element-instance")
         (Variable "$element"))
      (Implication
         (Variable "$predicate-instance")
         (Variable "$predicate"))
      (Evaluation
         (Variable "$predicate-instance")
         (List
            (Variable "$element-instance")))))

(define unary-predicate-speech-act-l2s-rewrite
   (Evaluation (stv 1 0.1)
      (Variable "$predicate")
      (Variable "$element")))

(define unary-predicate-speech-act-l2s-rule
   (Bind
      unary-predicate-speech-act-l2s-vardecl
      unary-predicate-speech-act-l2s-pattern
      unary-predicate-speech-act-l2s-rewrite))

;;;;;;;;;;;;;;;
;; S2L rules ;;
;;;;;;;;;;;;;;;

;; Rule to turn something like
;;
;; (Implication
;;    (Predicate "crazy")
;;    (Predicate "happy"))
;;
;; into
;;
;; (SetLink
;;    (Evaluation
;;       (Predicate "happy")
;;       (List
;;          (Concept "people")))
;;    (Inheritance
;;       (Concept "people")
;;       (Concept "crazy")))
;;
;; Mind that Predicate is turned into a Concept, that's normal.
;;
;; s2l stands for semantics to logic.

(define inheritance-to-evaluation-s2l-vardecl
   (VariableList
      (TypedVariable
         (Variable "$P")
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$Q")
         (Type "PredicateNode"))
      ;; The following is to minimize the probability that this rule
      ;; is gonna turn any Implication into a mess. We try to filter
      ;; in Implication inferred from r2l->l2s knowledge.
      (TypedVariable
         (Variable "$P-element-instance")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$P-element")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$P-instance")
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$P")
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$Q-element-instance")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Q-element")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$Q-instance")
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$Q")
         (Type "PredicateNode"))))

(define inheritance-to-evaluation-s2l-pattern
   (And
      (Implication
         (Variable "$P")
         (Variable "$Q"))
      ;; The following is to minimize the probability that this rule
      ;; is gonna turn any Implication into a mess. We try to filter
      ;; in Implication inferred from r2l->l2s knowledge.
      (Inheritance
         (Variable "$P-element-instance")
         (Variable "$P-element"))
      (Implication
         (Variable "$P-instance")
         (Variable "$P"))
      (Evaluation
         (Variable "$P-instance")
         (List
            (Variable "$P-element-instance")))
      (Inheritance
         (Variable "$Q-element-instance")
         (Variable "$Q-element"))
      (Implication
         (Variable "$Q-instance")
         (Variable "$Q"))
      (Evaluation
         (Variable "$Q-instance")
         (List
            (Variable "$Q-element-instance")))))

(define inheritance-to-evaluation-s2l-rewrite
   (ExecutionOutput
      (GroundedSchema "scm: inheritance-to-evaluation-s2l-formula")
      (List
         (Variable "$P")
         (Variable "$Q"))))

(define (inheritance-to-evaluation-s2l-formula P Q)
   (let ((P-name (cog-name P)))
       (Word "people")
       (Set
          (Evaluation
             Q
             (List
                (Concept "people")))
          (Inheritance
             (Concept "people")
             (Concept P-name)))))

(define inheritance-to-evaluation-s2l-rule
   (Bind
      inheritance-to-evaluation-s2l-vardecl
      inheritance-to-evaluation-s2l-pattern
      inheritance-to-evaluation-s2l-rewrite))

;;;;;;;;;;
;; Main ;;
;;;;;;;;;;


(define (pln-run)
  (define (pln-loop)
    ;; Apply l2s rules
    (cog-bind unary-predicate-speech-act-l2s-rule)

    ;; Apply Implication direct evaluation (and put the result in
    ;; pln-inferred-atoms state)
    (State pln-inferred-atoms (cog-bind implication-direct-evaluation-rule))

    ;; Log the current state
    (cog-logger-info "[PLN-Psi] pln-inferred-atoms = ~a"
                     (cog-incoming-set pln-inferred-atoms))

    ;; Sleep a bit, cause thinking is tiring
    (sleep 1)

    ;; Loop
    (pln-loop))
  (begin-thread (pln-loop)))

(pln-run)

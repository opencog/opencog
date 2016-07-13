;; Background PLN reasoning
;;
;; Very simplistic and hacky at the moment, loop of 2 rules
;;
;; 1. Preprocessing to turn r2l outputs into something that PLN can reason on
;;
;; 2. Limited induction reasoning
;;
;; The reasoner doesn't use the URE. Instead if merely applies the 2
;; rules one after the other in a loop.
;;
;; You may test it as following (wait for a couple seconds between
;; each command to be sure that the chatbot-psi and the pln-reasoner
;; have time to diggest them.
;;
;; guile -l main.scm
;; (chat "small cats are cute") ;; exemplar to help sureal to generate answer
;; (chat "Ben is crazy")
;; (chat "Ben is happy")
;; (chat "Eddie is funny")
;; (chat "Eddie is happy")
;; (chat "Robert is funny")
;; (chat "Robert is happy")
;; (chat "What do you know about happy?") ;; Answer: funny people are happy

(use-modules (opencog))
(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (srfi srfi-1))

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
;; (EvaluationLink
;;    (DefinedLinguisticPredicateNode "definite")
;;    (ListLink
;;       (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    )
;; )
;;
;; plus possibly (TBD)
;;
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
      ;; (TypedVariable
      ;;    (Variable "$specific-entity-instance")
      ;;    (Type "SpecificEntityNode"))
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
            (Variable "$element-instance")))
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
           (Variable "$element-instance")))))
      ;; (InheritanceLink
      ;;    (Variable "$specific-entity-instance")
      ;;    (Variable "$element"))))

(define unary-predicate-speech-act-l2s-rewrite
   (Evaluation (stv 1 0.1)
      (Variable "$predicate")
      (Variable "$element")))

(define unary-predicate-speech-act-l2s-rule
   (Bind
      unary-predicate-speech-act-l2s-vardecl
      unary-predicate-speech-act-l2s-pattern
      unary-predicate-speech-act-l2s-rewrite))

;;;;;;;;;;
;; Main ;;
;;;;;;;;;;

(define (pln-run)
  (define (pln-loop)
    ;; Apply l2s rules
    (let ((l2s-outputs (cog-bind unary-predicate-speech-act-l2s-rule)))
      (cog-logger-debug "[PLN-Reasoner] l2s-outputs = ~a" l2s-outputs))

    ;; Apply Implication direct evaluation (and put the result in
    ;; pln-inferred-atoms state)
    (let ((direct-eval-outputs (cog-bind implication-direct-evaluation-rule)))
      (State pln-inferred-atoms direct-eval-outputs)
      (cog-logger-debug "[PLN-Reasoner] pln-inferred-atoms = ~a"
                       direct-eval-outputs))

    ;; sleep a bit, to not overload the CPU too much
    (cog-logger-debug "[PLN-Reasoner] Sleep for a second")
    (sleep 1)

    ;; Loop
    (pln-loop))
  (begin-thread (pln-loop)))

(pln-run)

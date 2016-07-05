;; Inductive reasoning about crazy happy people

;;;;;;;;;;;;;;;;;;;;;;;
;; Load prerequisite ;;
;;;;;;;;;;;;;;;;;;;;;;;

;; Load modules
(use-modules (opencog query))

;; Load the chatbot
(add-to-load-path "../../../opencog/nlp/chatbot-psi")
(load-from-path "chatbot.scm")

;; Load PLN rule implication direct evaluation
(add-to-load-path "../../../opencog/pln/rules")
(load-from-path "implication-direct-evaluation-rule.scm")

;; Convenient fetchers
(use-modules (opencog query))
(define (get-parse-nodes)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$P")
                           (Type "ParseNode"))
                         (Variable "$P"))))

;; Get the r2l output
(define (get-set-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$S")
                           (Type "SetLink"))
                         (Variable "$S"))))

(define (get-wordinstance-nodes)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordInstanceNode"))
                         (Variable "$W"))))

;; Get all of them
(define (get-wordinstance-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordInstanceLink"))
                         (Variable "$W"))))

;; Get all of them
(define (get-wordsequence-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$W")
                           (Type "WordSequenceLink"))
                         (Variable "$W"))))

;; Get all of them
(define (get-lemma-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$L")
                           (Type "LemmaLink"))
                         (Variable "$L"))))

;; Get all of them
(define (get-reference-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$R")
                           (Type "ReferenceLink"))
                         (Variable "$R"))))

;; Get all of them
(define (get-interpretation-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$I")
                           (Type "InterpretationLink"))
                         (Variable "$I"))))

;; Get all of them
(define (get-execution-links)
  (cog-satisfying-set (Get
                         (TypedVariable
                           (Variable "$E")
                           (Type "ExecutionLink"))
                         (Variable "$E"))))

(chat "Ben is happy")
(chat "Ben is crazy")

(chat "Eddie is happy")
(chat "Eddie is crazy")

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
;; The 0.1 is just to convey the fact that we only have one piece of
;; evidence, ideally we would use count-to-confidence
;;
;; We call these rule l2s, which stands for logic to
;; semantics. Eventually maybe these can be turned into a rule-base,
;; using the URE, but for now it's more like a hack.

;;;;;;;;;;;;;;;;;;;;;;
;; Unary predicates ;;
;;;;;;;;;;;;;;;;;;;;;;

(define unary-predicate-speech-act-vardecl
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
         (Type "PredicateNode"))
      (TypedVariable
         (Variable "$interpretation")
         (Type "InterpretationNode"))))

(define unary-predicate-speech-act-pattern
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
      (Inheritance
         (Variable "$interpretation")
         (DefinedLinguisticConceptNode "DeclarativeSpeechAct"))))

(define unary-predicate-speech-act-rewrite
   (Evaluation (stv 1 0.1)
      (Variable "$predicate")
      (Variable "$element")))

(define unary-predicate-speech-act-rule
   (Bind
      unary-predicate-speech-act-vardecl
      unary-predicate-speech-act-pattern
      unary-predicate-speech-act-rewrite))

;; Apply all l2s rules
(cog-bind unary-predicate-speech-act-rule)

;; Apply Implication direct evaluation
(cog-bind implication-direct-evaluation-rule)

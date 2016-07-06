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

;; Apply l2s rules
(cog-bind unary-predicate-speech-act-l2s-rule)

;; Apply Implication direct evaluation
(cog-bind implication-direct-evaluation-rule)

;; Apply s2l rules
(cog-bind inheritance-to-evaluation-s2l-rule)

;; Apply sureal with the output s2l
(chat "small cats are cute")


;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; Sub-experiment, don't run it ;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;; Sub experiment turning the inferred knowledge into speech

(Word "people")
(Word "crazy")
(Word "happy")

;; Using inheritance directly (doesn't work)
(sureal (SetLink
      (InheritanceLink
         (ConceptNode "people@8107be66-4424-445b-9875-96fa5d3d3204")
         (ConceptNode "people" (stv 0.022727273 0.0012484394))
      )
      (InheritanceLink
         (ConceptNode "people@8107be66-4424-445b-9875-96fa5d3d3204")
         (ConceptNode "crazy")
      )
      (InheritanceLink
         (ConceptNode "people@8107be66-4424-445b-9875-96fa5d3d3204")
         (ConceptNode "happy")
      )
   )
)

;; Using predicate. Works, just need to add the following sentence
(chat "small cats are cute")
(Word "happy")
(Word "people")
(Word "crazy")

;; Wait a bit before pasting that because the chat needs a second to
;; get processed.
(sureal
   (SetLink
      (Evaluation
         (Predicate "happy")
         (List
            (Concept "people")))
      (Inheritance
         (Concept "people")
         (Concept "crazy"))))

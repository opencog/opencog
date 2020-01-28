;; Load PLN rule implication direct evaluation

(use-modules (opencog))
(use-modules (opencog nlp))
(use-modules (opencog pln))
(use-modules (opencog ure))
; FIXME: Doesn't return anything when confidence is low, don't use for now
;(load-from-path "opencog/pln/rules/implication-direct-evaluation.scm")

;;;;;;;;;;;;;;;;;;;;;
;; Extra knowledge ;;
;;;;;;;;;;;;;;;;;;;;;

;; Add knowledge about happy. The strenght is set low so that the new
;; inferred knowledge can get easily expressed.
;(Word "playful")
;(add-to-pln-inferred-atoms
; (Set
;    (Implication (stv 0.51 0.0001)
;       (Predicate "happy")
;       (Predicate "playful"))))
;(Word "funny")
;(add-to-pln-inferred-atoms
; (Set
;    (Implication (stv 0.51 0.0001)
;       (Predicate "funny")
;       (Predicate "happy"))))
;(Word "pleasant")
;(add-to-pln-inferred-atoms
; (Set
;    (Implication (stv 0.51 0.0001)
;       (Predicate "happy")
;       (Predicate "pleasant"))))
;(Word "free")
;(add-to-pln-inferred-atoms
; (Set
;    (Implication (stv 0.51 0.0001)
;       (Predicate "happy")
;       (Predicate "free"))))
;(Word "healthy")
;(add-to-pln-inferred-atoms
; (Set
;    (Implication (stv 0.51 0.0001)
;       (Predicate "happy")
;       (Predicate "healthy"))))
;
;;; Peter is happy
;(Evaluation (stv 1 0.1)
;   (Predicate "happy")
;   (Concept "Peter"))

;; This is required for SuReal to generate the answer
(nlp-parse "small cats are cute")

;-------------------------------------------------------------------------------
;;;;;;;;;;;;;;;
;; L2S rules ;;
;;;;;;;;;;;;;;;

;; Rule to turn something like
;;
;; (Evaluation
;;    (Predicate "name")
;;    (List
;;       (Concept <rec-id>)
;;       (Word <name>)))
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept <rec-id>)
;;       (Sentence <sentence-id-1>)))
;; (InheritanceLink
;;    (Sentence <sentence-id-1>)
;;    (Concept <sentiment-1>))
;; ...
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept <rec-id>)
;;       (Sentence <sentence-id-n>)))
;; (InheritanceLink
;;    (Sentence <sentence-id-n>)
;;    (Concept <sentiment-n>))
;;
;; into
;;
;; (Inheritance (stv s c)
;;    (Concept <name>)
;;    (Concept "happy"))
;;
;; where s in the number of positive sentences divided by the number of
;; sentences, and c is the number of sentences divided by 800.

(define sentiment-sentence-to-person-l2s-vardecl
   (VariableList
      (TypedVariable
         (Variable "$person")
         (Type "ConceptNode"))
      (TypedVariable
         (Variable "$name")
         (Type "WordNode"))
      (TypedVariable
         (Variable "$sentence")
         (Type "SentenceNode"))
      (TypedVariable
         (Variable "$sentiment")
         (Type "ConceptNode"))))

(define sentiment-sentence-to-person-l2s-pattern
   (And
      (Evaluation
         (Predicate "name")
         (List
            (Variable "$person")
            (Variable "$name")))
      (Evaluation
         (Predicate "say")
         (List
            (Variable "$person")
            (Variable "$sentence")))
      (Inheritance
         (Variable "$sentence")
         (Variable "$sentiment"))))

(define sentiment-sentence-to-person-l2s-rewrite
   (ExecutionOutput
      (GroundedSchema "scm: sentiment-sentence-to-person-l2s-formula")
      (List
         (Variable "$person")
         (Variable "$name"))))

(define sentiment-sentence-to-person-l2s-rule
   (Bind
      sentiment-sentence-to-person-l2s-vardecl
      sentiment-sentence-to-person-l2s-pattern
      sentiment-sentence-to-person-l2s-rewrite))

;; Return the number of sentences from person P tagged with sentiment S
;;
;; More specifically count the number of matches
;;
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       P
;;       (Sentence <sentence-1>)))
;; (InheritanceLink
;;    (Sentence <sentence-1>)
;;    S))
;; ...
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       P
;;       (Sentence <sentence-n>)))
;; (InheritanceLink
;;    (Sentence <sentence-n>)
;;    S)
(define (count-sentiment-sentences P S)
  (let* (
         (V (Variable "$sentence"))
         (vardecl (TypedVariable V (Type "SentenceNode")))
         (say-pattern (Evaluation (Predicate "say") (List P V)))
         (query-pattern (And say-pattern (InheritanceLink V S)))
         (query (Get vardecl query-pattern))
         (results (cog-execute! query)))
    (length (cog-outgoing-set results))))

(define (sentiment-sentence-to-person-l2s-formula Person Name)
  (let* (
         (K 800) ; parameter to convert from count to confidence
         ;; Count positive and negative sentences
         (pos-count (count-sentiment-sentences Person (Concept "Positive")))
         (neg-count (count-sentiment-sentences Person (Concept "Negative")))
         (total-count (+ pos-count neg-count))
         ;; Calculate strength and confidence
         (s (if (> total-count 0)
                (exact->inexact (/ pos-count total-count))
                0))
         (c (exact->inexact (/ total-count K))))
    ;; (cog-logger-info "[PLN-Reasoner] pos-count = ~a" pos-count)
    ;; (cog-logger-info "[PLN-Reasoner] pos-count = ~a" neg-count)
    (Evaluation (stv s c)
       (Predicate "happy")
       (Concept (cog-name Name)))))

;-------------------------------------------------------------------------------
;; Rule to turn something like
;;
;; (InheritanceLink
;;    (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    (ConceptNode "Ben" (stv 0.029411765 0.0012484394))
;; )
;; (ImplicationLink
;;    (PredicateNode "crazy@1d08ff8b-4149-4362-97ef-9103a307a879")
;;    (PredicateNode "crazy" (stv 0.25 0.0012484394))
;; )
;; (EvaluationLink
;;    (PredicateNode "crazy@1d08ff8b-4149-4362-97ef-9103a307a879")
;;    (ListLink
;;       (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    )
;; )
;; (EvaluationLink
;;    (DefinedLinguisticPredicateNode "definite")
;;    (ListLink
;;       (ConceptNode "Ben@b0f3845c-9cfb-4b39-99a6-131004f6203d")
;;    )
;; )
;;
;; into
;;
;; (EvaluationLink (stv 1 0.1)
;;    (Predicate "crazy")
;;    (ConceptNode "Ben"))
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
            (Variable "$element-instance")))
      (EvaluationLink
         (DefinedLinguisticPredicateNode "definite")
         (ListLink
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

;-------------------------------------------------------------------------------
; NOTE: Temporary replacement
(define implication-direct-evaluation-vardecl
  (VariableList
     (TypedVariable
        (Variable "$P")
        (Type "PredicateNode"))
     (TypedVariable
        (Variable "$Q")
        (Type "PredicateNode"))
     ;; Current hack to limit X as concepts
     (TypedVariable
        (Variable "$X")
        (Type "ConceptNode"))))

(define implication-direct-evaluation-pattern
  (And
     (Evaluation
        (Variable "$P")
        (Variable "$X"))
     (Evaluation
        (Variable "$Q")
        (Variable "$X"))
     (Not
        (Equal
           (Variable "$P")
           (Variable "$Q")))))

(define implication-direct-evaluation-rewrite
  (Implication
        (Variable "$P")
        (Variable "$Q")))

(define implication-direct-evaluation-rule
  (Bind
     implication-direct-evaluation-vardecl
     implication-direct-evaluation-pattern
     implication-direct-evaluation-rewrite))


;-------------------------------------------------------------------------------
(define (configure-pln-rbs-1)
    (define rb (ConceptNode "r2l-pln-1"))

    ; TODO: use pln-load-rules when move to new PLN API, see
    ; https://github.com/opencog/pln/blob/master/opencog/pln/README.md
    (load-from-path (pln-rule-type->filename "term/deduction"))
    (load-from-path (pln-rule-type->filename "wip/abduction"))

    ; NOTE: The number has no relevance in r2l-mode
    (ure-define-rbs rb 0)
    ; Not sure why
    (ure-set-fuzzy-bool-parameter rb "URE:attention-allocation" 0)

    ;; Add rules to rulebases.
    (ure-define-add-rule rb "rule1" sentiment-sentence-to-person-l2s-rule
      (stv 1 1))
    (ure-define-add-rule rb "rule2" unary-predicate-speech-act-l2s-rule
      (stv 1 1))
    (ure-define-add-rule rb "rule3" implication-direct-evaluation-rule
      (stv 1 1))

    ; Return the rule-base
    rb
)

;; Define rulebases
(define rb-trail-1 (configure-pln-rbs-1))

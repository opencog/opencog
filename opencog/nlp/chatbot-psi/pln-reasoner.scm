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
;;
;; ;; positive sentence, also helps sureal to generate answer
;; (mock-HEAD-chat "p-1" "Eddie" "small animals are cute")
;;
;; ;; positive sentences
;; (mock-HEAD-chat "p-2" "Ruiting" "birds are lovely")
;; (mock-HEAD-chat "p-3" "Ben" "dogs are awesome")
;; (mock-HEAD-chat "p-3" "Ben" "the multiverse is beautiful")
;;
;; ;; negative sentences
;; (mock-HEAD-chat "p-3" "Ben" "I hate to relax, it makes me nervous")
;;
;; ;; Statement about Ben
;; (mock-HEAD-chat "p-1" "Eddie" "Ben is crazy")
;;
;; ;; Question about happiness. Answer should be: crazy people are happy
;; (mock-HEAD-chat "p-1" "Eddie" "What do you know about happy?")

(use-modules (opencog))
(use-modules (opencog logger))
(use-modules (opencog query))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

;; (cog-logger-set-level! "debug")

;; Load PLN rule implication direct evaluation
(load-from-path "opencog/pln/rules/implication-direct-evaluation-rule.scm")


;;;;;;;;;;;;;;;
;; HEAD mock ;;
;;;;;;;;;;;;;;;

;; Code to mock the HEAD. Given a person id and its name, creates the
;; call chat and create the following
;;
;; (Evaluation
;;    (Predicate "name")
;;    (List
;;       (Concept person-id)
;;       (Concept name)))
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept person-id)
;;       (Sentence <sentence-id>)))
(define (mock-HEAD-chat person-id name message)
  (chat message)
  (sleep 1)                             ; you never know

  ;; Create name structure
  (Evaluation
     (Predicate "name")
     (List
        (Concept person-id)
        (Word name)))

  ;; Create say structure
  (let* ((sentence (cog-chase-link 'ListLink 'SentenceNode (Node message))))
    (Evaluation
       (Predicate "say")
       (List
          (Concept person-id)
          sentence))))

;;;;;;;;;;;;;;;;;;;;;
;; Extra knowledge ;;
;;;;;;;;;;;;;;;;;;;;;

;; Add knowledge about happy. The strenght is set low so that the new
;; inferred knowledge can get easily expressed.
(Word "playful")
(add-to-pln-inferred-atoms
 (Set
    (Implication (stv 0.51 0.0001)
       (Predicate "happy")
       (Predicate "playful"))))
(Word "funny")
(add-to-pln-inferred-atoms
 (Set
    (Implication (stv 0.51 0.0001)
       (Predicate "funny")
       (Predicate "happy"))))
(Word "pleasant")
(add-to-pln-inferred-atoms
 (Set
    (Implication (stv 0.51 0.0001)
       (Predicate "happy")
       (Predicate "pleasant"))))
(Word "free")
(add-to-pln-inferred-atoms
 (Set
    (Implication (stv 0.51 0.0001)
       (Predicate "happy")
       (Predicate "free"))))
(Word "healthy")
(add-to-pln-inferred-atoms
 (Set
    (Implication (stv 0.51 0.0001)
       (Predicate "happy")
       (Predicate "healthy"))))

;; Peter is happy
(Evaluation (stv 1 0.1)
   (Predicate "happy")
   (Concept "Peter"))

;;;;;;;;;;;;;;;
;; L2S rules ;;
;;;;;;;;;;;;;;;

;; Rule to put a name on the last sentence using:
;;
;; 1. The state of "last-recognized-face"
;;
;; (State
;;    (Anchor "last-recognized-face")
;;    (Concept <rec-id>))
;;
;; 2. The state of input-utterance-sentence
;;
;; (State
;;    input-utterance-sentence
;;    (Sentence <sentence-id>))
;;
;; To produce
;;
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept <rec-id>)
;;       (Sentence <sentence-1>)))
;;
;; In addition to that it also produces
;;
;; (Evaluation
;;    (Predicate "name")
;;    (List
;;       (Concept <rec-id>
;;       (Word <name>)))
;;
;; with specific ids and names.
(define (put-name-on-the-last-sentence)
  (let ((last-sentence-id (get-last-sentence-id))
        (last-rec-id (get-last-rec-id))
        ;; rec-ids and names
        (rec-id-1 "20839")
        (name-1 "Louis"))
    (cog-logger-debug "[PLN-Reasoner] last-sentence-id = ~a" last-sentence-id)
    (cog-logger-debug "[PLN-Reasoner] last-rec-id = ~a" last-rec-id)
    (StateLink (AnchorNode "last-recognized-face") (ConceptNode "20839"))
    (Evaluation
       (Predicate "name")
          (List
             (Concept rec-id-1)
             (Word name-1)))
    (if (or (null? last-sentence-id) (null? last-rec-id))
        '()
        (Evaluation
           (Predicate "say")
           (List
              last-rec-id
              last-sentence-id)))))

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
         (results (cog-satisfying-set query)))
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
    ;; (cog-logger-debug "[PLN-Reasoner] pos-count = ~a" pos-count)
    ;; (cog-logger-debug "[PLN-Reasoner] pos-count = ~a" neg-count)
    (Evaluation (stv s c)
       (Predicate "happy")
       (Concept (cog-name Name)))))

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


;; Define rulebases
;XXX Why separate rulebases?
(define rb1 (ConceptNode "rb1"))
(ure-define-rbs rb1 1)
; Not sure why
(ure-set-fuzzy-bool-parameter rb1 "URE:attention-allocation" 0)

(define rb2 (ConceptNode "rb2"))
(ure-define-rbs rb2 1)
(ure-set-fuzzy-bool-parameter rb2 "URE:attention-allocation" 0)

(define rb3 (ConceptNode "rb3"))
(ure-define-rbs rb3 1)
(ure-set-fuzzy-bool-parameter rb3 "URE:attention-allocation" 0)

;; Add rules to rulebases.
(ure-define-add-rule rb1 "rule1" sentiment-sentence-to-person-l2s-rule .8)
(ure-define-add-rule rb2 "rule2" unary-predicate-speech-act-l2s-rule .8)
(ure-define-add-rule rb3 "rule3" implication-direct-evaluation-rule .8)

;;;;;;;;;;
;; Main ;;
;;;;;;;;;;
; TODO: Remove this loop by integrating the pln-demo to openpsi
(define enable-pln-loop #f)
(define (pln-running?) enable-pln-loop)

(define pln-loop-count 0)
(define (pln-get-loop-count) pln-loop-count)

(define (pln-loop)
  ;; Apply l2s rules
  (let (
        (name-on-last-sentence (put-name-on-the-last-sentence))
        (sentiment-sentence-to-person-l2s-results
         ;(cog-bind sentiment-sentence-to-person-l2s-rule)
         (cog-fc (SetLink) rb1 (SetLink))
         )
        (unary-predicate-speech-act-l2s-results
         ;(cog-bind unary-predicate-speech-act-l2s-rule)
         (cog-fc (SetLink) rb2 (SetLink))
         ))
    ;; (cog-logger-debug "[PLN-Reasoner] StateLinks = ~a" (cog-get-atoms 'StateLink))
    ;; (cog-logger-debug "[PLN-Reasoner] PredicateNodes = ~a" (map cog-incoming-set (cog-get-atoms 'PredicateNode)))
    (cog-logger-debug "[PLN-Reasoner] name-on-last-sentence = ~a" name-on-last-sentence)
    (cog-logger-debug "[PLN-Reasoner] sentiment-sentence-to-person-l2s-results = ~a" sentiment-sentence-to-person-l2s-results)
    (cog-logger-debug "[PLN-Reasoner] unary-predicate-speech-act-l2s-results = ~a" unary-predicate-speech-act-l2s-results))

  ;; Apply Implication direct evaluation (and put the result in
  ;; pln-inferred-atoms state)
  (let* ((direct-eval-results (cog-fc (SetLink) rb3 (SetLink)) );(cog-bind implication-direct-evaluation-rule))
        ;; Filter only inferred result containing "happy". This is a
        ;; temporary hack to make it up for the lack of attentional
        ;; allocation
        (must-contain (list (Predicate "happy")))
        (ff (lambda (x) (lset<= equal? must-contain (cog-get-all-nodes x))))
        (filtered-results (filter ff (cog-outgoing-set direct-eval-results))))
    (add-to-pln-inferred-atoms (Set filtered-results)))

  (cog-logger-debug "[PLN-Reasoner] pln-inferred-atoms = ~a"
                    (search-inferred-atoms))

  ;; sleep a bit, to not overload the CPU too much
  (cog-logger-debug "[PLN-Reasoner] Sleep for a second")
  (set! pln-loop-count (+ pln-loop-count 1))
  (sleep 1)

  ;; Loop
  (if enable-pln-loop (pln-loop))
)

(define (pln-run)
    (if (not (pln-running?))
        (begin
            (set! enable-pln-loop #t)
            (begin-thread (pln-loop))))
)

(define (pln-halt) (set! enable-pln-loop #f))

; Start pln loop
(pln-run)

(use-modules (opencog) (opencog nlp) (opencog nlp oc) (opencog nlp relex2logic))

(load "states.scm")
(load "utils.scm")

;-------------------------------------------------------------------------------
; Useful functions for the contexts

(define (is-utterance-type? speechact)
    (Satisfaction (And
        (State input-utterance-sentence (Variable "$s"))
        (Parse (Variable "$parse") (Variable "$s"))
        (Interpretation (Variable "$interp") (Variable "$parse"))
        (Inheritance (Variable "$interp") speechact)
    ))
)

(define (setup-done? anchor)
    (Not (Equal (Set setup-not-done) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s")))))
)

(define (process-not-started? anchor)
    (Equal (Set default-state) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s"))))
)

(define (process-finished? anchor)
    (Equal (Set process-finished) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State anchor (Variable "$s"))))
)

(define (any-result? anchor)
    (Not (Or
        (Equal (Set setup-not-done) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
        (Equal (Set default-state) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
        (Equal (Set no-result) (Get
            (TypedVariable (Variable "$f") (Type "ConceptNode"))
                (State anchor (Variable "$f"))))
    ))
)

(define (no-result? anchor)
    (Equal (Set no-result) (Get
        (TypedVariable (Variable "$r") (Type "ConceptNode"))
            (State anchor (Variable "$r"))))
)

(define (check-aiml-reply num-node)
    (let* ((tv (cog-tv (aiml-get-selected-rule)))
           (conf (cog-tv-confidence tv))
           (threshold (cog-number num-node)))
        (if (> conf threshold)
            (stv 1 1)
            (stv 0 1)
        )
    )
)

(define (check-fuzzy-reply num-node)
    (if (> (cog-number (car
            (cog-chase-link 'StateLink 'NumberNode fuzzy-reply-conf)))
                (cog-number num-node))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (long-time-elapsed)
    (if (> (- (current-time) (string->number (get-input-time)))
            (cog-number (gar (cog-execute!
                (Get (TypedVariable (Variable "$t") (Type "TimeNode"))
                    (State max-waiting-time (Variable "$t")))))))
        (stv 1 1)
        (stv 0 1)
    )
)

(define (check-words . target-words)
    ; target-words is a list of WordNodes
    (if (nil? (filter (lambda (w) (list? (member w target-words)))
            (cog-outgoing-set (get-input-word-list))))
        (stv 0 1)
        (stv 1 1)
    )
)

(define rsg-input "")
(define (check-theme target-words)
    (define input-text (cog-name (get-input-text-node)))
    (define matched-keywords '())

    (for-each
        (lambda (w)
            (let ((r1 (regexp-exec (make-regexp
                          (string-append "\\b" w "\\b") regexp/icase) input-text))
                  (r2 (regexp-exec (make-regexp
                          (string-append "\\w+\\s+\\b" w "\\b") regexp/icase) input-text))
                  (r3 (regexp-exec (make-regexp
                          (string-append "\\b" w "\\b\\s+\\w+") regexp/icase) input-text)))
                (if (not (equal? #f r1))
                    (set! matched-keywords (append matched-keywords (list w))))
                (if (not (equal? #f r2))
                    (set! matched-keywords (append matched-keywords (list (match:substring r2)))))
                (if (not (equal? #f r3))
                    (set! matched-keywords (append matched-keywords (list (match:substring r3)))))
            )
        )
        target-words
    )

    (if (> (length matched-keywords) 0)
        (begin
            (set! rsg-input (list-ref matched-keywords (random (length matched-keywords))))
            (stv 1 1))
        (stv 0 1)
    )
)

(define (check-pkd-words)
    (check-theme pkd-relevant-words)
)

(define (check-blog-words)
    (check-theme blog-relevant-words)
)

(define (check-kurzweil-words)
    (check-theme kurzweil-relevant-words)
)

;-------------------------------------------------------------------------------

(Define
    (DefinedPredicate "is-input-utterance?")
    (Not (Equal (Set no-input-utterance)
        (Get (TypedVariable (Variable "$x") (Type "ConceptNode"))
            (State input-utterance (Variable "$x")))))
)

(Define
    (DefinedPredicate "input-type-is-declarative?")
    (is-utterance-type? (DefinedLinguisticConcept "DeclarativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-imperative?")
    (is-utterance-type? (DefinedLinguisticConcept "ImperativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-interrogative?")
    (is-utterance-type? (DefinedLinguisticConcept "InterrogativeSpeechAct"))
)

(Define
    (DefinedPredicate "input-type-is-truth-query?")
    (is-utterance-type? (DefinedLinguisticConcept "TruthQuerySpeechAct"))
)

(Define
    (DefinedPredicate "input-is-a-question?")
    (Or (DefinedPredicate "input-type-is-interrogative?")
        (DefinedPredicate "input-type-is-truth-query?"))
)

(Define
    (DefinedPredicate "input-is-about-the-robot?")
    (Evaluation (GroundedPredicate "scm: check-words")
        (List (Word "you") (Word "your") (Word "yours") (Word "robot")))
)

(Define
    (DefinedPredicate "asking-how-robot-feels?")
    (Evaluation (GroundedPredicate "scm: check-emotion-state-inquiry") (List))
)

(define (check-emotion-state-inquiry)
    (define inputwords (get-input-word-list))
    (if (not (nil? inputwords))
        (if (list? (member (cog-outgoing-set inputwords) (map text2wordnodes
                    (list
                        "how are you feeling"
                        "how are you"
                        "how do you feel"))))
            (stv 1 1)
            (stv 0 1)
        )
    )
)

; Convert text string to list of WordNodes
(define (text2wordnodes text)
    (map (lambda (word) (Word word)) (string-split text #\ ))
)


; Essentially equivalent to "is-input-utterance", as the states
; will be reset after giving a reply by default
(Define
    (DefinedPredicate "has-not-replied-anything-yet?")
    (DefinedPredicate "is-input-utterance?")
)

(Define
    (DefinedPredicate "fuzzy-not-started?")
    (process-not-started? fuzzy)
)

(Define
    (DefinedPredicate "fuzzy-finished?")
    (process-finished? fuzzy)
)

(Define
    (DefinedPredicate "is-fuzzy-reply?")
    (any-result? fuzzy-reply)
)

(Define
    (DefinedPredicate "fuzzy-reply-is-declarative?")
    (Equal (Set (DefinedLinguisticConcept "DeclarativeSpeechAct"))
        (Get (TypedVariable (Variable "$s") (Type "DefinedLinguisticConceptNode"))
            (State fuzzy-reply-type (Variable "$s"))))
)

(Define
    (DefinedPredicate "is-fuzzy-reply-good?")
    (Evaluation (GroundedPredicate "scm: check-fuzzy-reply") (List (Number .5)))
)

(Define
    (DefinedPredicate "no-other-fast-reply?")
    ; TODO: May want to check more than time elapsed
    (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List))
)

(Define
    (DefinedPredicate "aiml-not-started?")
    (process-not-started? aiml)
)

(Define
    (DefinedPredicate "aiml-finished?")
    (process-finished? aiml)
)

; Number being passed is the confidence threshold
(Define
    (DefinedPredicate "is-aiml-reply-good?")
    (Evaluation (GroundedPredicate "scm: check-aiml-reply") (List (Number .5)))
)

(Define
    (DefinedPredicate "no-result-from-other-sources?")
    (And (process-finished? duckduckgo)
         (process-finished? wolframalpha)
         (process-finished? pln-qa)
         (no-result? duckduckgo-answer)
         (no-result? wolframalpha-answer)
         (no-result? pln-answers))
)

(Define
    (DefinedPredicate "no-good-fast-answer?")
    (Or (DefinedPredicate "no-result-from-other-sources?")
        (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List)))
)

(Define
    (DefinedPredicate "is-aiml-reply?")
    (any-result? aiml-reply)
)

(Define
    (DefinedPredicate "chatscript-not-started?")
    (process-not-started? chatscript)
)

(Define
    (DefinedPredicate "chatscript-finished?")
    (process-finished? chatscript)
)

(Define
    (DefinedPredicate "is-chatscript-reply?")
    (any-result? chatscript-reply)
)

(Define
    (DefinedPredicate "duckduckgo-not-started?")
    (process-not-started? duckduckgo)
)

(Define
    (DefinedPredicate "duckduckgo-finished?")
    (process-finished? duckduckgo)
)

(Define
    (DefinedPredicate "is-duckduckgo-answer?")
    (any-result? duckduckgo-answer)
)

(Define
    (DefinedPredicate "is-wolframalpha-ready?")
    (setup-done? wolframalpha)
)

(Define
    (DefinedPredicate "wolframalpha-not-started?")
    (process-not-started? wolframalpha)
)

(Define
    (DefinedPredicate "wolframalpha-finished?")
    (process-finished? wolframalpha)
)

(Define
    (DefinedPredicate "is-wolframalpha-answer?")
    (any-result? wolframalpha-answer)
)

(Define
    (DefinedPredicate "is-openweathermap-ready?")
    (setup-done? openweathermap)
)

(Define
    (DefinedPredicate "openweathermap-not-started?")
    (process-not-started? openweathermap)
)

(Define
    (DefinedPredicate "is-weather-related?")
    (Evaluation (GroundedPredicate "scm: check-words") (List (Word "weather")))
)

(Define
    (DefinedPredicate "openweathermap-finished?")
    (process-finished? openweathermap)
)

(Define
    (DefinedPredicate "is-openweathermap-answer?")
    (any-result? openweathermap-answer)
)

; Usually ChatScript is faster than the random sentence generator, so if the input has
; some of the keywords that will trigger either the PKD or Kurzweil generator, do not
; reply until the generator has finished or timeout
(Define
    (DefinedPredicate "has-random-sentence-generator-done-with-the-keywords?")
    (Or (SequentialAnd
            (DefinedPredicate "has-pkd-related-words?")
            (DefinedPredicate "random-pkd-sentence-generated?"))
        (SequentialAnd
            (DefinedPredicate "has-kurzweil-related-words?")
            (DefinedPredicate "random-kurzweil-sentence-generated?"))
        (DefinedPredicate "no-other-fast-reply?")
    )
)

(Define
    (DefinedPredicate "no-random-sentence-generator-keywords?")
    (Not (Or
        (DefinedPredicate "has-pkd-related-words?")
        (DefinedPredicate "has-kurzweil-related-words?")))
)

(Define
    (DefinedPredicate "is-random-pkd-sentence-generator-ready?")
    (setup-done? random-pkd-sentence-generator)
)

(Define
    (DefinedPredicate "is-random-blogs-sentence-generator-ready?")
    (setup-done? random-blogs-sentence-generator)
)

(Define
    (DefinedPredicate "is-random-kurzweil-sentence-generator-ready?")
    (setup-done? random-kurzweil-sentence-generator)
)

(Define
    (DefinedPredicate "random-pkd-sentence-generator-not-started?")
    (process-not-started? random-pkd-sentence-generator)
)

(Define
    (DefinedPredicate "random-blogs-sentence-generator-not-started?")
    (process-not-started? random-blogs-sentence-generator)
)

(Define
    (DefinedPredicate "random-kurzweil-sentence-generator-not-started?")
    (process-not-started? random-kurzweil-sentence-generator)
)

(Define
    (DefinedPredicate "random-pkd-sentence-generated?")
    (any-result? random-pkd-sentence-generated)
)

(Define
    (DefinedPredicate "random-blogs-sentence-generated?")
    (any-result? random-blogs-sentence-generated)
)

(Define
    (DefinedPredicate "random-kurzweil-sentence-generated?")
    (any-result? random-kurzweil-sentence-generated)
)

(Define
    (DefinedPredicate "has-pkd-related-words?")
    (Evaluation (GroundedPredicate "scm: check-pkd-words") (List))
)

(Define
    (DefinedPredicate "has-blog-related-words?")
    (Evaluation (GroundedPredicate "scm: check-blog-words") (List))
)

(Define
    (DefinedPredicate "has-kurzweil-related-words?")
    (Evaluation (GroundedPredicate "scm: check-kurzweil-words") (List))
)

(Define
    (DefinedPredicate "chatbot-eva-not-started?")
    (process-not-started? chatbot-eva)
)

(Define
    (DefinedPredicate "chatbot-eva-finished?")
    (process-finished? chatbot-eva)
)

(Define
    (DefinedPredicate "going-to-do-the-action?")
    (Not (Equal (Set no-result) (Get
        (TypedVariable (Variable "$s") (Type "ConceptNode"))
            (State chatbot-eva-action (Variable "$s")))))
)

(Define
    (DefinedPredicate "don't-know-how-to-do-it?")
    (Or (Evaluation (GroundedPredicate "scm: long-time-elapsed") (List))
        (Equal (Set no-result) (Get
            (TypedVariable (Variable "$s") (Type "ConceptNode"))
                (State chatbot-eva-action (Variable "$s")))))
)

(Define
    (DefinedPredicate "emotion-state-not-started?")
    (process-not-started? emotion-state)
)

(Define
    (DefinedPredicate "emotion-state-finished?")
    (process-finished? emotion-state)
)

(Define
    (DefinedPredicate "is-emotion-state-reply?")
    (any-result? emotion-state-reply)
)

(Define
    (DefinedPredicate "is-asking-about-how-many-visible-faces")
    (Satisfaction
        (VariableList
            (var-decl "$sent" "SentenceNode")
            (var-decl "$parse" "ParseNode")
            (var-decl "$how-inst" "WordInstanceNode")
            (var-decl "$many-inst" "WordInstanceNode")
            (var-decl "$ppl-inst" "WordInstanceNode"))
        (And
            (State input-utterance-sentence (Variable "$sent"))
            (parse-of-sent "$parse" "$sent")
            (word-in-parse "$how-inst" "$parse")
            (word-in-parse "$many-inst" "$parse")
            (word-in-parse "$ppl-inst" "$parse")
            (Evaluation (LinkGrammarRelationship "H")
                (List (Variable "$how-inst") (Variable "$many-inst")))
            (word-pos "$ppl-inst" "noun")
            (Choice (Lemma (Variable "$ppl-inst") (Word "people"))
                    (Lemma (Variable "$ppl-inst") (Word "face")))
        )
    )
)

; Keyword: "done showing"
(Define
    (DefinedPredicate "is-asked-to-stop-demo?")
    (Satisfaction
        (VariableList
            (var-decl "$sent" "SentenceNode")
            (var-decl "$parse" "ParseNode")
            (var-decl "$done" "WordInstanceNode")
            (var-decl "$showing" "WordInstanceNode")
        )
        (And
            (State input-utterance-sentence (Variable "$sent"))
            (parse-of-sent "$parse" "$sent")
            (word-in-parse "$done" "$parse")
            (word-in-parse "$showing" "$parse")
            (Reference (Variable "$done") (Word "done"))
            (Reference (Variable "$showing") (Word "showing"))
            (Evaluation (LinkGrammarRelationship "Pg")
                (List (Variable "$done") (Variable "$showing")))
        )
    )
)

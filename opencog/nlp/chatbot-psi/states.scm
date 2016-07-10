(use-modules (opencog))

;-------------------------------------------------------------------------------
; Keep track of the states

(define (chat-prefix node_name) (string-append "Chatbot: " node_name))

(define input-utterance (Anchor (chat-prefix "InputUtterance")))
(define no-input-utterance (Concept (chat-prefix "NoInputUtterance")))
(State input-utterance no-input-utterance)

(define input-parse (Anchor (chat-prefix "InputParse")))
(define parse-succeeded (Concept (chat-prefix "ParseSucceeded")))
(define parse-failed (Concept (chat-prefix "ParseFailed")))
(State input-parse no-input-utterance)

(define default-state (Concept (chat-prefix "DefaultState")))
(define process-started (Concept (chat-prefix "ProcessStarted")))
(define process-finished (Concept (chat-prefix "ProcessFinished")))
(define no-result (Concept (chat-prefix "NoResult")))

(define aiml-replies (Anchor (chat-prefix "AIMLReplies")))
(define aiml-search (Anchor (chat-prefix "AIMLSearch")))
(State aiml-replies default-state)
(State aiml-search default-state)

(define fuzzy-replies (Anchor (chat-prefix "FuzzyReplies")))
(define fuzzy-match (Anchor (chat-prefix "FuzzyMatch")))
(State fuzzy-replies default-state)
(State fuzzy-match default-state)

(define fuzzy-answers (Anchor (chat-prefix "FuzzyAnswers")))
(define fuzzy-qa (Anchor (chat-prefix "FuzzyQA")))
(State fuzzy-answers default-state)
(State fuzzy-qa default-state)

(define duckduckgo-answers (Anchor (chat-prefix "DuckDuckGoAnswers")))
(define duckduckgo-search (Anchor (chat-prefix "DuckDuckGoSearch")))
(State duckduckgo-answers default-state)
(State duckduckgo-search default-state)

(define wolframalpha-answers (Anchor (chat-prefix "WolframAlphaAnswers")))
(define wolframalpha-search (Anchor (chat-prefix "WolframAlphaSearch")))
(State wolframalpha-answers default-state)
(State wolframalpha-search default-state)

(define random-sentence-generator (Anchor (chat-prefix "RandomSentenceGenerator")))
(define random-sentence-generated (Anchor (chat-prefix "RandomSentenceGenerated")))
(State random-sentence-generator default-state)
(State random-sentence-generated default-state)

(define chatbot-eva (Anchor (chat-prefix "ChatbotEva")))
(define sent-to-chatbot-eva (Concept (chat-prefix "SentToChatbotEva")))
(define no-action-taken (Concept (chat-prefix "NoActionTaken")))
(State chatbot-eva default-state)

;; PLN states
(define pln-answers (Anchor (chat-prefix "PLNAnswers")))
(define pln-qa (Anchor (chat-prefix "PLNQA")))
(define pln-inferred-atoms (Anchor (chat-prefix "PLNInferredAtoms")))
(State pln-answers default-state)
(State pln-qa default-state)
(State pln-inferred-atoms default-state) ;; should not be reset

;-------------------------------------------------------------------------------

(define (reset-all-states)
    (State input-utterance no-input-utterance)
    (State aiml-replies default-state)
    (State aiml-search default-state)
    (State fuzzy-replies default-state)
    (State fuzzy-match default-state)
    (State fuzzy-answers default-state)
    (State fuzzy-qa default-state)
    (State duckduckgo-answers default-state)
    (State duckduckgo-search default-state)
    (State wolframalpha-answers default-state)
    (State wolframalpha-search default-state)
    (State random-sentence-generator default-state)
    (State random-sentence-generated default-state)
    (State chatbot-eva default-state)
    (State pln-answers default-state)
    (State pln-qa default-state)
)

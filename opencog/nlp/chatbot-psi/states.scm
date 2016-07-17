(use-modules (opencog))

;-------------------------------------------------------------------------------
; Keep track of the states

(define (chat-prefix node_name) (string-append "Chatbot: " node_name))

(define input-utterance (Anchor (chat-prefix "InputUtterance")))
(define input-utterance-sentence (Anchor (chat-prefix "InputUtteranceSentence")))
(define input-utterance-text (Anchor (chat-prefix "InputUtteranceText")))
(define input-utterance-words (Anchor (chat-prefix "InputUtteranceWords")))
(define no-input-utterance (Concept (chat-prefix "NoInputUtterance")))
(State input-utterance no-input-utterance)
(State input-utterance-sentence no-input-utterance)  ; No need to be reset
(State input-utterance-text no-input-utterance)  ; No need to be reset
(State input-utterance-words no-input-utterance)  ; No need to be reset

(define input-parse (Anchor (chat-prefix "InputParse")))
(define parse-succeeded (Concept (chat-prefix "ParseSucceeded")))
(define parse-failed (Concept (chat-prefix "ParseFailed")))
(State input-parse no-input-utterance)

(define default-state (Concept (chat-prefix "DefaultState")))
(define process-started (Concept (chat-prefix "ProcessStarted")))
(define process-finished (Concept (chat-prefix "ProcessFinished")))
(define setup-not-done (Concept (chat-prefix "SetupNotDone")))
(define no-result (Concept (chat-prefix "NoResult")))

(define aiml (Anchor (chat-prefix "AIML")))
(define aiml-reply (Anchor (chat-prefix "AIMLReply")))
(State aiml default-state)
(State aiml-reply default-state)

(define fuzzy (Anchor (chat-prefix "Fuzzy")))
(define fuzzy-reply (Anchor (chat-prefix "FuzzyReply")))
(define fuzzy-reply-type (Anchor (chat-prefix "FuzzyReplyType")))
(define fuzzy-reply-conf (Anchor (chat-prefix "FuzzyReplyConfidence")))
(State fuzzy default-state)
(State fuzzy-reply default-state)
(State fuzzy-reply-type default-state)
(State fuzzy-reply-conf default-state)

(define duckduckgo (Anchor (chat-prefix "DuckDuckGo")))
(define duckduckgo-answer (Anchor (chat-prefix "DuckDuckGoAnswer")))
(State duckduckgo default-state)
(State duckduckgo-answer default-state)

(define wolframalpha (Anchor (chat-prefix "WolframAlpha")))
(define wolframalpha-answer (Anchor (chat-prefix "WolframAlphaAnswer")))
(State wolframalpha setup-not-done)  ; An AppID is required to use WolframAlpha
(State wolframalpha-answer default-state)

(define random-sentence-generator (Anchor (chat-prefix "RandomSentenceGenerator")))
(define random-sentence-generated (Anchor (chat-prefix "RandomSentenceGenerated")))
(State random-sentence-generator setup-not-done)  ; Need to do (markov-setup ...) first
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

(define (reset-all-chatbot-states)
    (State input-utterance no-input-utterance)
    (State aiml default-state)
    (State aiml-reply default-state)
    (State fuzzy default-state)
    (State fuzzy-reply default-state)
    (State fuzzy-reply-type default-state)
    (State fuzzy-reply-conf default-state)
    (State duckduckgo default-state)
    (State duckduckgo-answer default-state)
    (if has-wolframalpha-setup
        (State wolframalpha default-state))
    (State wolframalpha-answer default-state)
    (if has-markov-setup
        (State random-sentence-generator default-state))
    (State random-sentence-generated default-state)
    (State chatbot-eva default-state)
    (State pln-answers default-state)
    (State pln-qa default-state)
)

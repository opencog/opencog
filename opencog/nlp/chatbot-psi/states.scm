(use-modules (opencog) (opencog nlp) (opencog nlp oc))

;-------------------------------------------------------------------------------
; Keep track of the states

(define (chat-prefix node_name) (string-append "Chatbot: " node_name))

(define input-utterance (Anchor (chat-prefix "InputUtterance")))
(define-public input-utterance-sentence (Anchor (chat-prefix "InputUtteranceSentence")))
(define-public input-utterance-text (Anchor (chat-prefix "InputUtteranceText")))
(define-public input-utterance-words (Anchor (chat-prefix "InputUtteranceWords")))
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
(define-public process-started (Concept (chat-prefix "ProcessStarted")))
(define-public process-finished (Concept (chat-prefix "ProcessFinished")))
(define setup-not-done (Concept (chat-prefix "SetupNotDone")))
(define-public no-result (Concept (chat-prefix "NoResult")))

(define max-waiting-time (Anchor (chat-prefix "MaxWaitingTime")))
(State max-waiting-time (Time 3))

(define aiml (Anchor (chat-prefix "AIML")))
(define aiml-reply (Anchor (chat-prefix "AIMLReply")))
(State aiml default-state)
(State aiml-reply default-state)

(define chatscript (Anchor (chat-prefix "ChatScript")))
(define chatscript-reply (Anchor (chat-prefix "ChatScriptReply")))
(State chatscript default-state)
(State chatscript-reply default-state)

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

(define openweathermap (Anchor (chat-prefix "OpenWeatherMap")))
(define openweathermap-answer (Anchor (chat-prefix "OpenWeatherMapAnswer")))
(State openweathermap setup-not-done)  ; An AppID is required to use OpenWeatherMap
(State openweathermap-answer default-state)

(define random-pkd-sentence-generator (Anchor (chat-prefix "RandomPKDSentenceGenerator")))
(define random-blogs-sentence-generator (Anchor (chat-prefix "RandomBlogsSentenceGenerator")))
(define random-kurzweil-sentence-generator (Anchor (chat-prefix "RandomBlogsSentenceGenerator")))
(define random-pkd-sentence-generated (Anchor (chat-prefix "RandomPKDSentenceGenerated")))
(define random-blogs-sentence-generated (Anchor (chat-prefix "RandomBlogsSentenceGenerated")))
(define random-kurzweil-sentence-generated (Anchor (chat-prefix "RandomBlogsSentenceGenerated")))
; Need to do (markov-setup ...) once for both generators
(State random-pkd-sentence-generator setup-not-done)
(State random-blogs-sentence-generator setup-not-done)
(State random-kurzweil-sentence-generator setup-not-done)
(State random-pkd-sentence-generated default-state)
(State random-blogs-sentence-generated default-state)
(State random-kurzweil-sentence-generated default-state)

(define chatbot-eva (Anchor (chat-prefix "ChatbotEva")))
(define chatbot-eva-action (Anchor (chat-prefix "ChatbotEvaAction")))
(State chatbot-eva default-state)
(State chatbot-eva-action default-state)

;; PLN states
(define-public pln-answers (Anchor (chat-prefix "PLNAnswers")))
(define-public pln-qa (Anchor (chat-prefix "PLNQA")))
(define-public pln-inferred-atoms (Anchor (chat-prefix "PLNInferredAtoms")))
(State pln-answers default-state)
(State pln-qa default-state)

(define emotion-state (Anchor (chat-prefix "EmotionState")))
(define emotion-state-reply (Anchor (chat-prefix "EmotionStateReply")))
(State emotion-state default-state)
(State emotion-state-reply default-state)

;-------------------------------------------------------------------------------

(define (reset-all-chatbot-states)
    (State input-utterance no-input-utterance)
    (State aiml default-state)
    (State aiml-reply default-state)
    (State chatscript default-state)
    (State chatscript-reply default-state)
    (State fuzzy default-state)
    (State fuzzy-reply default-state)
    (State fuzzy-reply-type default-state)
    (State fuzzy-reply-conf default-state)
    (State duckduckgo default-state)
    (State duckduckgo-answer default-state)
    (if has-wolframalpha-setup
        (State wolframalpha default-state))
    (State wolframalpha-answer default-state)
    (if has-openweathermap-setup
        (State openweathermap default-state))
    (State openweathermap-answer default-state)
    (if has-markov-setup (begin
        (State random-pkd-sentence-generator default-state)
        (State random-blogs-sentence-generator default-state)
        (State random-kurzweil-sentence-generator default-state)))
    (State random-pkd-sentence-generated default-state)
    (State random-blogs-sentence-generated default-state)
    (State random-kurzweil-sentence-generated default-state)
    (State chatbot-eva default-state)
    (State chatbot-eva-action default-state)
    (State pln-answers default-state)
    (State pln-qa default-state)
    (State emotion-state default-state)
    (State emotion-state-reply default-state)
)


(use-modules (opencog))
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))

; Prime the atomspace with content that sureal can use to generate
; sentences.  We do this here, by parsing these sentences, during
; the module load. It might be better, long-run, to fetch this from
; a database.

; "Where are you looking?"
(nlp-parse "I am looking to the left")
(nlp-parse "I am looking to the right")
(nlp-parse "I am looking up")
(nlp-parse "I am looking upward")
(nlp-parse "I am looking downward")
(nlp-parse "I am looking leftwards")
(nlp-parse "I am looking rightwards")
(nlp-parse "I am looking forward")

(nlp-parse "I am looking at you")

; "What are you doing?"
(nlp-parse "I am smiling")
(nlp-parse "I am frowning")
(nlp-parse "I am not doing anything")

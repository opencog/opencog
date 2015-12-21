
; For users who are not aware of readline ...
(use-modules (ice-9 readline))
(activate-readline)

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp) (opencog nlp chatbot-eva))
(use-modules (opencog nlp relex2logic))

(load-r2l-rulebase)

(start-cogserver "../lib/opencog-chatbot.conf")

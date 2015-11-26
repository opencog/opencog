

; Stuff actually needed to get the chatbot running...
(use-modules (opencog) (opencog cogserver))

(use-modules (opencog nlp) (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))

(load-r2l-rulebase)

(start-cogserver "../lib/opencog-chatbot.conf")

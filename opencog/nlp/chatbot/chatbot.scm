(define-module (opencog nlp chatbot))

(use-modules (opencog)
             (opencog atom-types)
             (opencog rule-engine)
             (opencog nlp)
             (opencog nlp fuzzy)
             (opencog nlp microplanning)
             (opencog nlp relex2logic))

(define relex-server-host "127.0.0.1")
(define relex-server-port 4444)

(load "chatbot/processing-utils.scm")
(load "chatbot/process-query.scm")

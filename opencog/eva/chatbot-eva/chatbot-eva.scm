;
; The Hanson Robotics Eva chatbot module.
;
(define-module (opencog nlp chatbot-eva))

(use-modules (opencog) (opencog nlp chatbot))

(load "chatbot-eva/self-model.scm")
(load "chatbot-eva/knowledge.scm")
(load "chatbot-eva/semantics.scm")
(load "chatbot-eva/imperative-rules.scm")
(load "chatbot-eva/imperative.scm")
(load "chatbot-eva/load-sureal.scm")
(load "chatbot-eva/model-query.scm")
(load "chatbot-eva/bot-api.scm")

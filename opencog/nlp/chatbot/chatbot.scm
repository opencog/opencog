;
; The scheme chatbot module.
;
(define-module (opencog nlp chatbot))

(load "chatbot/chat-utils.scm")
(load "chatbot/bot-api.scm")

; Temporary debug support
(init-trace "/tmp/chatty")
(trace-msg "--------------- Start tracing -------------------\n")

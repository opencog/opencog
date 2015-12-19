;
; The scheme chatbot module.
;
(define-module (opencog nlp chatbot))

(load "chatbot/chat-utils.scm")
(load "chatbot/eva.scm")
(load "chatbot/process-query.scm")

; Temporary debug support
(init-trace "/tmp/chatty")
(trace-msg "--------------- Start tracing -------------------\n")

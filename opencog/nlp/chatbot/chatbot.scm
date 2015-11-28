;
; The scheme chatbot module.
;
(define-module (opencog nlp chatbot))

; User-modifiable config paramters.
(define relex-server-host "127.0.0.1")
(define relex-server-port 4444)

(load "chatbot/chat-utils.scm")
(load "chatbot/process-query.scm")

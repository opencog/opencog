; This is file is used for specifying function used for choosing a focus-set

(use-modules (opencog))
(use-modules (opencog atom-types))
(use-modules (opencog nlp))
(use-modules (opencog nlp chatbot))
(use-modules (opencog nlp relex2logic))

; ----------------------------------------------------------------------------
(define-public (pln-get-nlp-inputs sn-list)
"
  pln-get-nlp-inputs SN-LIST

  Given a list of SentenceNodes SN-LIST, it returns a list with their
  abstracted-version of r2l outputs.
"
    (append-map
        (lambda (sent-node)
            (get-abstract-version (car (sent-get-interp sent-node))))
        sn-list)
)

; ----------------------------------------------------------------------------
; Time based means of choosing focus-set content, from nlp-pipeline.
; ----------------------------------------------------------------------------
(define start-of-chat-recording (Anchor "start-of-chat-recording"))

(define-public (pln-record-current-time)
"
  Record the current-time.
"
    (set! start-of-chat-recording (current-time))
)

(define-public (pln-get-recorded-time)
"
  Returns the time recorded by (pln-record-current-time).
"
    start-of-chat-recording
)

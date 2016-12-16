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

  Given a list of SentenceNodes ,sn-list, it returns a list with atoms that
  can be inferred on.
"
    (append-map
        (lambda (sent-node)
            (let* ((r2l-outputs (interp-get-r2l-outputs
                        (car (sent-get-interp sent-node))))
                  (nodes (delete-duplicates
                        (append-map cog-get-all-nodes r2l-outputs)))
                  (cn-and-pn (delete-duplicates
                        (filter
                            (lambda (x)
                                (or (equal? 'ConceptNode (cog-type x))
                                    (equal? 'PredicateNode (cog-type x))))
                            nodes))))
                (append
                    r2l-outputs
                    (append-map
                        ; For sog-hack-decomposition-rule
                        (lambda (x) (cog-incoming-by-type x 'ReferenceLink))
                        cn-and-pn))
            ))
        sn-list)
)

; ----------------------------------------------------------------------------
; Time based means of choosing focus-set content, from nlp-pipeline.
; ----------------------------------------------------------------------------
(define start-of-chat-recording 0)

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

(use-modules (opencog) (rnrs bytevectors))

(define server "127.0.0.1")
(define port 1024)
(define username "opencog")
(define bot-name "rose")

(define-public (setup-chatscript robot)
    (shutdown (car (send-to-chatscript (string-append ":build " robot))) 2)
)

(define (send-to-chatscript msg)
    (define bv (make-bytevector 10000))
    (define s (socket AF_INET SOCK_STREAM 6))

    (connect s AF_INET (inet-pton AF_INET server) port)

    (send s (string->utf8 (string-append username "\0" bot-name "\0" msg "\0")))

    (cons s bv)
)

(define (call-chatscript msg)
    (define prtn (send-to-chatscript msg))
    (define s (car prtn))
    (define bv (cdr prtn))

    (recv! s bv)

    ; TODO: Store in the AtomSpace
    (display (utf8->string bv))
    (newline)

    (shutdown s 2)
)

(setup-chatscript "han")
(call-chatscript "what is your name")

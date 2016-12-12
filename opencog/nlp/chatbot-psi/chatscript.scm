(use-modules (opencog) (rnrs bytevectors))

; ChatScript server config
(define cs-server "127.0.0.1")
(define cs-port 1024)
(define cs-username "opencog")
(define cs-bot-name "rose")

(define (send-to-chatscript msg)
    (define bv (make-bytevector 10000))
    (define s (socket AF_INET SOCK_STREAM 6))

    (connect s AF_INET (inet-pton AF_INET cs-server) cs-port)
    (send s (string->utf8 (string-append cs-username "\0" cs-bot-name "\0" msg "\0")))
    (cons s bv)
)

(define (call-chatscript)
    (State chatscript process-started)

    (begin-thread
        ; Returns a pair -- the socket and the byte-vector (the reply generated)
        (define rtn (send-to-chatscript (cog-name (get-input-text-node))))
        (define s (car rtn))
        (define bv (cdr rtn))

        (recv! s bv)

        ; TODO: Store in the AtomSpace
        (display (utf8->string bv))
        (newline)

        ; TODO: Parse the reply?
        (State chatscript-reply (List (map Word (string-split (utf8->string bv) #\ ))))
        (State chatscript process-finished)

        (shutdown s 2)
    )
)

(define-public (setup-chatscript robot)
    (shutdown (car (send-to-chatscript (string-append ":build " robot))) 2)
)

; TODO: Move to HR
(setup-chatscript "han")

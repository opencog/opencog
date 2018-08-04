(use-modules (opencog) (rnrs bytevectors))

; ChatScript server config
(define cs-server "127.0.0.1")
(define cs-port 1024)
(define cs-username "opencog")
(define cs-bot-name "")

; Send a message to the ChatScript server
(define (send-to-chatscript msg)
    (define bv (make-bytevector 10000))
    (define s (socket AF_INET SOCK_STREAM 6))

    (connect s AF_INET (inet-pton AF_INET cs-server) cs-port)
    (send s (string->utf8 (string-append cs-username "\0" cs-bot-name "\0" msg "\0")))
    (cons s bv)
)

; Send the input to the ChatScript server and store the reply into the AtomSpace
(define (call-chatscript)
    (State chatscript process-started)

    (begin-thread
        ; Returns a pair -- the socket and the byte-vector (the reply generated)
        (define rtn (send-to-chatscript (cog-name (get-input-text-node))))
        (define s (car rtn))
        (define bv (cdr rtn))
        (define br (recv! s bv))
        (define reply (utf8->string bv))
        (define regex "\\[callback=.+\\] ")

        ; Remove any null char before doing regexp-substitute
        (set! reply (string-delete (lambda (c) (char=? #\nul c)) reply))

        ; Sometimes the reply may have some unwanted string at the beginning
        ; e.g. "[callback=3000 ] Greetings!"
        ; Ignore the first part of it
        (set! reply (regexp-substitute/global #f regex reply 'pre "" 'post))

        ; TODO: Parse the reply?
        (State chatscript-reply (List (map Word (string-split reply #\ ))))
        (State chatscript process-finished)

        (shutdown s 2)
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

; Initial setup for the ChatScript server, to load different set of rules for
; different robots
(define-public (chatscript-setup robot)
    (shutdown (car (send-to-chatscript (string-append ":build " robot))) 2)
)

; Change the IP of the server
(define-public (set-chatscript-server ip)
    (set! cs-server ip)
)

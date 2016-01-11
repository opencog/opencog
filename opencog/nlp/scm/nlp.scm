
(define-module (opencog nlp))

(use-modules (opencog) (opencog atom-types))

; User-modifiable config paramters.
(define relex-server-host "127.0.0.1")
(define relex-server-port 4444)

(define-public (relex-server-host! ip)
"
  Set the relex-server's ip. By default it is 127.0.0.1.

  ip:
  - A string specifying the ip
"
    (if (not (string? ip)) (error "Expected the ip to be a string, got: " ip))
    (set! relex-server-host ip)
)

(define-public (relex-server-port! port)
"
  Set the port number for the relex-server. By default it is 4444

  port:
  - A number specifying the port number
"
    (if (not (number? port))
        (error "Expected the ip to be a number, got:" port))
    (set! relex-server-port port)
)

; Load various parts....
(load "nlp/nlp-utils.scm")
(load "nlp/disjunct-list.scm")
(load "nlp/processing-utils.scm")

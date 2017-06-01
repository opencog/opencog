(use-modules (opencog))
(use-modules (ice-9 threads) (srfi srfi-1) (sxml simple) (web client) (web response))

;-------------------------------------------------------------------------------
; AppID for Wolfram|Alpha Webservice API
(define wa-appid "")
(define has-wolframalpha-setup #f)

; AppID for OpenWeatherMap
(define owm-appid "")
(define has-openweathermap-setup #f)

(define-public (set-wa-appid id)
    (set! wa-appid id)
    (State wolframalpha default-state)
    (set! has-wolframalpha-setup #t)
)

(define-public (set-owm-appid id)
    (set! owm-appid id)
    (State openweathermap default-state)
    (set! has-openweathermap-setup #t)
)

(define (ask-duckduckgo)
    (State duckduckgo process-started)

    (begin-thread
        ; TODO: Do something better for getting the first sentence of a paragraph, though
        ; it isn't that critical here
        (define (get-first-sentence str)
            (define default-length 50)

            (if (< (string-length str) default-length)
                (substring str 0 (string-index str #\.))
                (substring str 0 (+ default-length
                    (string-index (substring str default-length) #\.)))
            )
        )

        (define query (string-downcase (cog-name (get-input-text-node))))
        (define url (string-append "http://api.duckduckgo.com/?q=" query "&format=xml"))
        (define body (xml->sxml (response-body-port (http-get url #:streaming? #t))))
        (define resp (car (last-pair body)))
        (define abstract
            (find (lambda (i)
                (and (pair? i) (equal? 'Abstract (car i)))) resp))

        (if (equal? (length abstract) 1)
            (State duckduckgo-answer no-result)
            (let* ((ans (car (cdr abstract)))
                   (first-sent (get-first-sentence ans))
                   (ans-in-words (string-split first-sent #\ ))
                  )
                (State duckduckgo-answer (List (map Word ans-in-words)))
            )
        )

        (State duckduckgo process-finished)
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (ask-wolframalpha)
    (if (not (equal? wa-appid ""))
        (begin
            (State wolframalpha process-started)
            (begin-thread
                (define query (string-downcase (cog-name (get-input-text-node))))
                (define query-no-spaces (regexp-substitute/global #f " " query 'pre "+" 'post))
                (define url (string-append
                    "http://api.wolframalpha.com/v2/query?appid=" wa-appid
                        "&input=" query-no-spaces "&format=plaintext"))
                (define body (xml->sxml (response-body-port (http-get url #:streaming? #t))))
                (define resp (car (last-pair body)))

                (define ans
                    (find (lambda (i)
                            (and (pair? i)
                                 (equal? 'pod (car i))
                                 (equal? 'title (car (cadr (cadr i))))
                                 ; The tags we are looking for
                                 (not (equal? (member (cadr (cadr (cadr i)))
                                     (list "Result" "Definition" "Definitions"
                                           "Basic definition" "Basic information"))
                                     #f))))
                    resp)
                )

                (if (equal? ans #f)
                    (State wolframalpha-answer no-result)
                    (let* ((text-ans (cadr (cadddr (cadddr ans))))
                           ; Remove '(', ')', and '|' from the answer, if any
                           (cleaned-ans (string-trim (string-filter
                               (lambda (c) (not (or (char=? #\( c)
                                                    (char=? #\) c)
                                                    (char=? #\| c)))) text-ans)))
                           ; Remove newline and split them into words
                           (ans-lines (string-split cleaned-ans #\newline))
                           (ans-in-words (append-map (lambda (l) (string-split l #\ )) ans-lines)))
                        ; Turn the answer into WordNodes, ignore the empty strings
                        (State wolframalpha-answer (List (map Word
                            (remove (lambda (w) (equal? w "")) ans-in-words))))
                    )
                )

                (State wolframalpha process-finished)
            )
        )
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

(define (ask-weather)
    (if (not (equal? owm-appid ""))
        (begin
            (State openweathermap process-started)
            (begin-thread
                (define ip-api "http://freegeoip.net/xml/")
                (define ip-body (xml->sxml (response-body-port (http-get ip-api #:streaming? #t))))
                (define ip-resp (car (last-pair ip-body)))
                (define country-code
                    (cadr (find (lambda (i) (and (pair? i) (equal? 'CountryCode (car i)))) ip-resp)))

                (define owm-url (string-append "http://api.openweathermap.org/data/2.5/weather?q="
                    country-code "&appid=" owm-appid "&mode=xml&units=imperial"))
                (define owm-body (xml->sxml (response-body-port (http-get owm-url #:streaming? #t))))
                (define owm-resp (car (last-pair owm-body)))
                (define temp (find (lambda (d) (and (pair? d) (equal? 'temperature (car d)))) owm-resp))
                (define humidity (find (lambda (d) (and (pair? d) (equal? 'humidity (car d)))) owm-resp))
                (define weather (find (lambda (d) (and (pair? d) (equal? 'weather (car d)))) owm-resp))
                (define temp-val (cadr (cadr (cadr temp))))
                (define humidity-val (cadr (cadr (cadr humidity))))
                (define weather-val (cadr (cadr (cadr weather))))

                (if (equal? weather-val #f)
                    (State openweathermap-answer no-result)
                    (State openweathermap-answer (List (append
                        (list (Word "it's"))
                        (map Word (string-split weather-val #\ ))
                        (list (Word "temperature") (Word temp-val) (Word "fahrenheit"))
                        (list (Word "humidity") (Word humidity-val) (Word "percent"))
                    )))
                )

                (State openweathermap process-finished)
            )
        )
    )

    ; Return for the GroundedSchemaNode
    (Set)
)

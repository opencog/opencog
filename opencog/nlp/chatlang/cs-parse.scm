(use-modules (ice-9 rdelim))
(use-modules (rnrs io ports))
(use-modules (system base lalr))
(use-modules (ice-9 regex))

(define (display-token token)
  (if (lexical-token? token)
    (format #t
      "lexical-category = ~a, lexical-source = ~a, lexical-value = ~a\n"  (lexical-token-category token)
      (lexical-token-source token)
      (lexical-token-value token)
    )
    (begin
      (display "passing on \n")
      token)
  )
)

(define (get-source-location port)
  (make-source-location
    (port-filename port)
    (port-line port)
    (port-column port)
    (false-if-exception (ftell port))
    #f
  )
)

(define (show-location loc)
  (format #f "line = ~a & column = ~a"
    (source-location-line loc)
    (source-location-column loc)
  )
)

(define (get-concept-name str)
  (define match (string-match "^~[a-zA-Z]+" str))
  (if match
    (cons (match:substring match) (match:suffix match))
    (error "Issue calling get-concept-name on " str)
  )
)

(define (tokeniz str location)
  (define current-match '())
  (define (tokenization-result token location a-pair)
    (cons
      (make-lexical-token token location (car a-pair))
      (cdr a-pair)))

  (define (has-match? pattern str)
    (let ((match (string-match pattern str)))
      (if match
        (begin (set! current-match match) #t)
        #f
      )))

  (cond
    ((has-match? "^\\(" str)
        (cons
          (make-lexical-token 'LPAREN location #f)
          (match:suffix current-match)))
    ((has-match? "^\\)" str)
        (cons
          (make-lexical-token 'RPAREN location #f)
          (match:suffix current-match)))
    ; Chatscript declarations
    ((has-match? "^concept:" str)
        (tokenization-result 'CONCEPT location
          (get-concept-name (string-trim (match:suffix current-match)))))
    ((has-match? "^topic:" str)
        (format #t  ">>lexer topic @ ~a\n" (show-location location))
        (make-lexical-token 'TOPIC location "a topic"))
    ((has-match? "^\r" str)
        (format #t  ">>lexer cr @ ~a\n" (show-location location))
        (make-lexical-token 'CR location #f))
    ((string=? "" str)
        (format #t  ">>lexer newline @ ~a\n" (show-location location))
        (cons (make-lexical-token 'NEWLINE location #f) ""))
    ((has-match? "^#!" str) ; This should be checked always before #
        (format #t  ">>lexer sample input @ ~a\n" (show-location location))
        (make-lexical-token 'SAMPLE_INPUT location #f))
    ((has-match? "^#" str)
        (format #t  ">>lexer comment @ ~a\n" (show-location location))
        (make-lexical-token 'COMMENT location #f))
    ; Chatscript rules
    ((has-match? "^[s?u]:" str)
        (format #t ">>lexer responders @ ~a\n" (show-location location))
        (make-lexical-token 'RESPONDERS location "a responders"))
    ((has-match? "^[c-j]:" str)
        (format #t ">>lexer rejoinder @ ~a\n" (show-location location))
        (make-lexical-token 'REJOINDERS location "a rejoinders"))
    ((has-match? "^[rt]:" str)
        (format #t ">>lexer gambit @ ~a\n" (show-location location))
        (make-lexical-token 'GAMBIT location "a gambit"))
    ((has-match? "^[a-zA-Z]+" str) ; This should always be at the end.
        (format #t ">>lexer literal @ ~a\n" (show-location location))
        (cons
          (make-lexical-token 'LITERAL location
            (match:substring current-match))
          (match:suffix current-match)))
    (else
      (format #t ">>lexer non @ ~a\n" (show-location location))
      (make-lexical-token 'NotDefined location str))
  )
)

(define (cs-lexer port)
  (let ((cs-line ""))
    (lambda ()
      (if (string=? "" cs-line) (set! cs-line (read-line port)))
      (let ((port-location (get-source-location port)))
        (if (eof-object? cs-line)
          '*eoi*
          (let ((result (tokeniz (string-trim-both cs-line) port-location)))
            (if (pair? result)
              (begin
                (set! cs-line (cdr result))
                (car result))
              (error
                  (format #f "Tokenizer issue => STRING = ~a, LOCATION = ~a"
                      (lexical-token-value result)
                      (lexical-token-source result)))
            )))))
  )
)

(define cs-parser
  (lalr-parser
    ; Token (aka terminal symbol) definition
    (LPAREN RPAREN NEWLINE CR CONCEPT TOPIC RESPONDERS REJOINDERS GAMBIT
      NotDefined COMMENT SAMPLE_INPUT LITERAL
    )

    ; Parsing rules (aka nonterminal symbols)
    (lines
      (lines line) : (format #t "lines is ~a\n" $2)
      (line) : (format #t "line is ~a\n" $1)
    )

    (line
      (declarations) : (format #f "declarations= ~a\n" $1)
      (rules) : (format #f "rules= ~a\n" $1)
      (CR) : #f
      (NotDefined) : (display-token $1)
      (NEWLINE) : #f
      (COMMENT) : #f
      (SAMPLE_INPUT) : #f ; TODO replace with a tester function
    )

    (declarations
      (CONCEPT LPAREN literals RPAREN) : (display-token (string-append $1 " = " $3))
      (TOPIC) : (display-token $1)
    )

    (literals
      (literals LITERAL) :  (display-token (string-append $1 " " $2))
      (LITERAL) : (display-token $1)
    )

    (rules
      (RESPONDERS) : (display-token $1)
      (REJOINDERS) : (display-token $1)
      (GAMBIT) : (display-token $1)
    )
  )
)

; Test lexer
(define (test-lexer lexer)
  (define temp (lexer))
  (if (lexical-token? temp)
    (begin
      ;(display-token temp)
      (test-lexer lexer))
    #f
  )
)

(define (make-cs-lexer file-path)
  (cs-lexer (open-file-input-port file-path))
)

;(test-lexer (make-cs-lexer "test.top"))

; Test parser
(cs-parser (make-cs-lexer "test.top") error)
(display "\n--------- Finished a file ---------\n")

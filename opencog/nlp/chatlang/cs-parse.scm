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

(define (tokeniz str location)
  (define current-match '())
  (define (has-match? pattern str)
    (let ((match (string-match pattern str)))
      (if match
        (begin (set! current-match match) #t)
        #f
      )))

  (cond
    ; Chatscript declarations
    ((has-match? "concept:" str)
        (format #t ">>lexer concept @ ~a\n" (show-location location))
        (make-lexical-token 'CONCEPT location "a concept"))
    ((has-match? "topic:" str)
        (format #t  ">>lexer topic @ ~a\n" (show-location location))
        (make-lexical-token 'TOPIC location "a topic"))
    ((has-match? "^\r" str)
        (format #t  ">>lexer cr @ ~a\n" (show-location location))
        (make-lexical-token 'CR location #f))
    ((string=? "" str)
        (format #t  ">>lexer newline @ ~a\n" (show-location location))
        (make-lexical-token 'NEWLINE location #f))
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
    (else
      (format #t ">>lexer non @ ~a\n" (show-location location))
      (make-lexical-token 'NotDefined location "a not-defined"))
  )
)

(define (cs-lexer port)
  (lambda ()
    (let* ((line (read-line port))
          (port-location (get-source-location port)))
      (if (eof-object? line)
        '*eoi*
        (tokeniz (string-trim-both line) port-location)
      )
    )
  )
)

(define cs-parser
  (lalr-parser
    ; Token (aka terminal symbol) definition
    (LPAREN RPAREN NEWLINE CR CONCEPT TOPIC RESPONDERS REJOINDERS GAMBIT
      NotDefined COMMENT SAMPLE_INPUT
    )

    ; Parsing rules (aka nonterminal symbol)
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
      (CONCEPT) : (display-token $1)
      (TOPIC) : (display-token $1)
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

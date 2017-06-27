(use-modules (ice-9 rdelim))
(use-modules (rnrs io ports))
(use-modules (system base lalr))
(use-modules (ice-9 regex))

(define (display-token token)
"
  This is used as a place holder.
"
  (if (lexical-token? token)
    (format #t
      "lexical-category = ~a, lexical-source = ~a, lexical-value = ~a\n"  (lexical-token-category token)
      (lexical-token-source token)
      (lexical-token-value token)
    )
    (begin
      ;(display "passing on \n")
      token)
  )
)

(define (get-source-location port column)
  (make-source-location
    (port-filename port)
    (port-line port)
    column
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

  ;NOTE The matching must be done starting from the most specific to the
  ; the broadest regex patterns.
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
        (tokenization-result 'CONCEPT location
          (get-concept-name (string-trim (match:suffix current-match)))))
    ((has-match? "^\r" str)
        (format #t  ">>lexer cr @ ~a\n" (show-location location))
        (make-lexical-token 'CR location #f))
    ((string=? "" str)
        (cons (make-lexical-token 'NEWLINE location #f) ""))
    ((has-match? "^#!" str) ; This should be checked always before #
        ; TODO Add tester function for this
        (cons (make-lexical-token 'SAMPLE_INPUT location #f) ""))
    ((has-match? "^#" str)
        (cons (make-lexical-token 'COMMENT location #f) ""))
    ; Chatscript rules
    ((has-match? "^[s?u]:" str)
        (format #t ">>lexer responders @ ~a\n" (show-location location))
        (cons
          (make-lexical-token 'RESPONDERS location #f)
          (match:suffix current-match)))
    ((has-match? "^[a-q]:" str)
        (cons
          (make-lexical-token 'REJOINDERS location #f)
          (match:suffix current-match)))
    ((has-match? "^[rt]:" str)
        (cons
          (make-lexical-token 'GAMBIT location #f)
          (match:suffix current-match)))
    ((has-match? "^_" str)
        (cons
          (make-lexical-token '_ location #f)
          (match:suffix current-match)))
    ((has-match? "^~n" str)
        (cons
          (make-lexical-token '~n location #f)
          (match:suffix current-match)))
    ((has-match? "^~" str); Must be after other '~
        (cons
          (make-lexical-token '~ location #f)
          (match:suffix current-match)))
    ((has-match? "^," str)
        (cons
          (make-lexical-token 'COMMA location ",")
          (match:suffix current-match)))
    ((has-match? "^\\[" str)
        (cons
          (make-lexical-token 'LSBRACKET location #f)
          (match:suffix current-match)))
    ((has-match? "^]" str)
        (cons
          (make-lexical-token 'RSBRACKET location #f)
          (match:suffix current-match)))
    ((has-match? "^<<" str)
        (cons
          (make-lexical-token '<< location #f)
          (match:suffix current-match)))
    ((has-match? "^>>" str)
        (cons
          (make-lexical-token '>> location #f)
          (match:suffix current-match)))
    ((has-match? "^\\*" str)
        (cons
          (make-lexical-token '* location #f)
          (match:suffix current-match)))
    ((has-match? "^[0-9]+" str)
        (cons
          (make-lexical-token 'NUM location
            (match:substring current-match))
          (match:suffix current-match)))
    ((has-match? "^['!?.a-zA-Z-]+" str) ; This should always be at the end.
        (cons
          (make-lexical-token 'LITERAL location
            (match:substring current-match))
          (match:suffix current-match)))
    (else
      (format #t ">>Tokenizer non @ ~a\n" (show-location location))
      (make-lexical-token 'NotDefined location str))
  )
)

(define (cs-lexer port)
  (let ((cs-line "") (initial-line ""))
    (lambda ()
      (if (string=? "" cs-line)
        (begin
          (set! cs-line (read-line port))
          (set! initial-line cs-line)
        ))
        (format #t ">>>>>>>>>>> line being processed ~a\n" cs-line)
      (let ((port-location (get-source-location port
                              (string-contains initial-line cs-line))))
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
      NotDefined COMMENT SAMPLE_INPUT LITERAL WHITESPACE COMMA NUM
      _ * << >>
      ~n ; Range-restricted Wildcards
      ~ ; Concepts
       LSBRACKET RSBRACKET ; Square Brackets []
    )

    ; Parsing rules (aka nonterminal symbols)
    (lines
      (lines line) : (format #t "lines is ~a -- ~a\n" $1 $2)
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
      (CONCEPT LPAREN patterns RPAREN) : (display-token (string-append $1 " = " $3))
      (TOPIC LPAREN patterns RPAREN) : (display-token (string-append $1 " = " $3))
    )

    (rules
      (RESPONDERS a-literal LPAREN patterns RPAREN patterns) :
        (display-token (format #f "rejoiner(~a -> ~a)" $3 $5))
      (RESPONDERS LPAREN patterns RPAREN patterns) :
        (display-token (format #f "rejoiner(~a -> ~a)" $3 $5))
      (REJOINDERS LPAREN patterns RPAREN patterns) :
        (display-token (format #f "rejoiner(~a -> ~a)" $3 $5))
      (GAMBIT patterns) : (display-token (string-append "gambit = " $2))
    )

    (patterns ; TODO: Give this a better name.
      (literals) : (display-token $1)
      (choices) : (display-token $1)
      (unordered-matchings) : (display-token $1)
    )

    (literals
      (literals a-literal) :  (display-token (string-append $1 " " $2))
      (a-literal) :  (display-token $1)
    )

    (a-literal
      (LITERAL) : (display-token $1)
      (_ LITERAL) : (display-token (string-append "underscore_fn->" $2))
      (~ LITERAL) : (display-token (string-append "get_concept_fn->" $2))
      (* ~ NUM) : (display-token (string-append "range-restricted-*->~" $3))
      (LITERAL ~n) : (display-token (string-append $1 "<-range_wildecard"))
      (LITERAL COMMA) : (display-token (string-append $1 " " $2))
    )

    (choices
      (choices choice) : (display-token (string-append $1 " " $2))
      (choice) : (display-token $1)
    )

    (choice
      (LSBRACKET literals RSBRACKET) : (display-token (string-append "choices_fn->" $2))
    )

    (unordered-matchings
      (unordered-matchings unordered-matching) :
          (display-token (string-append $1 " " $2))
      (unordered-matching) : (display-token $1)
    )

    (unordered-matching
      (<< patterns >>) : (display-token (string-append "unordered-matchings->" $2))
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

(use-modules (ice-9 rdelim))
(use-modules (ice-9 regex))
(use-modules (ice-9 getopt-long))
(use-modules (rnrs io ports))
(use-modules (system base lalr))


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
"
  This returns a record of the source location. It doesn't correct for
  whitespaces that may get trimmed off.
"
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

(define (tokeniz str location)
  (define current-match '())
  (define (has-match? pattern str)
    (let ((match (string-match pattern str)))
      (if match
        (begin (set! current-match match) #t)
        #f
      )))

  (define (result:suffix token location value)
    ; Returns a lexical-token
    (cons
      (make-lexical-token token location value)
      (match:suffix current-match)))

  (define (command-pair)
    ; Return command string.
    (let ((match (string-match "~" (match:substring current-match))))
      ; The suffix is the command and the prefix is the argument
      (cons (match:suffix match) (string-trim (match:prefix match)))
    ))

  ; NOTE
  ; 1. The matching must be done starting from the most specific to the
  ;    the broadest regex patterns.
  ; 2. #f is given as a token value for those cases which don't have any
  ;    semantic values.
  (cond
    ((has-match? "^[ ]*\\(" str) (result:suffix 'LPAREN location #f))
    ((has-match? "^[ ]*\\)" str) (result:suffix 'RPAREN location #f))
    ; Chatscript declarations
    ((has-match? "^[ ]*concept:" str) (result:suffix 'CONCEPT location #f))
    ((has-match? "^[ ]*topic:" str) (result:suffix 'TOPIC location #f))
    ; The token value for 'CR is empty-string so as to handle multiline
    ; rules.
    ((has-match? "^[ ]*\r" str) (result:suffix 'CR location ""))
    ; FIXME: This is not really newline.
    ((string=? "" str) (cons (make-lexical-token 'NEWLINE location #f) ""))
    ((has-match? "^[ \t]*#!" str) ; This should be checked always before #
      ; TODO Add tester function for this
      (cons (make-lexical-token 'SAMPLE_INPUT location #f) ""))
    ((has-match? "^[ \t]*#" str)
      (cons (make-lexical-token 'COMMENT location #f) ""))
    ; Chatscript rules
    ((has-match? "^[ ]*[s?u]:" str)
      (result:suffix 'RESPONDERS location
        (substring (string-trim (match:substring current-match)) 0 1)))
    ((has-match? "^[ \t]*[a-q]:" str)
      (result:suffix 'REJOINDERS location
        (substring (string-trim (match:substring current-match)) 0 1)))
    ((has-match? "^[ ]*[rt]:" str) (result:suffix 'GAMBIT location #f))
    ((has-match? "^[ ]*_[0-9]" str)
      (result:suffix 'MVAR location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*_" str) (result:suffix 'VAR location #f))
    ; For dictionary keyword sets
    ((has-match? "^[ ]*[a-zA-Z]+~[a-zA-Z1-9]+" str)
      (result:suffix 'LITERAL~COMMAND location (command-pair)))
    ; Range-restricted Wildcards.
    ; TODO Maybe replace with dictionary keyword sets then process it on action?
    ((has-match? "^[ ]*\\*~[1-9]+" str)
      (result:suffix '*~n location
        (substring (string-trim (match:substring current-match)) 2)))
    ((has-match? "^[ ]*~[a-zA-Z_]+" str)
      (result:suffix 'ID location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*," str) (result:suffix 'COMMA location ","))
    ((has-match? "^[ ]*\\^" str) (result:suffix '^ location #f))
    ((has-match? "^[ ]*\\[" str) (result:suffix 'LSBRACKET location #f))
    ((has-match? "^[ ]*]" str) (result:suffix 'RSBRACKET location #f))
    ((has-match? "^[ ]*<<" str) (result:suffix '<< location #f))
    ((has-match? "^[ ]*>>" str) (result:suffix '>> location #f))
    ; This should follow <<
    ((has-match? "^[ ]*<" str) (result:suffix '< location #f))
    ; This should follow >>
    ((has-match? "^[ ]*>" str) (result:suffix '> location #f))
    ((has-match? "^[ ]*\"" str) (result:suffix 'DQUOTE location "\""))
    ((has-match? "^[ ]*\\*" str) (result:suffix '* location "*"))
    ((has-match? "^[ ]*!" str) (result:suffix 'NOT location #f))
    ((has-match? "^[ ]*[?]" str) (result:suffix '? location "?"))
    ; This should always be near the end, because it is broadest of all.
    ((has-match? "^[ \t]*['.,_!0-9a-zA-Z-]+" str)
      (result:suffix 'LITERAL location
        (string-trim (match:substring current-match))))
    ; This should always be after the above so as to match words like "3D".
    ((has-match? "^[ ]*[0-9]+" str)
      (result:suffix 'NUM location
        (string-trim (match:substring current-match))))
    ; NotDefined token is used for errors only and there shouldn't be any rules.
    (else (cons (make-lexical-token 'NotDefined location str) ""))
  )
)

(define (cs-lexer port)
  (let ((cs-line "") (initial-line ""))
    (lambda ()
      (if (string=? "" cs-line)
        (begin
          (format #t "\n-------------------------- line ~a \n" (port-line port))
          (set! cs-line (read-line port))
          (set! initial-line cs-line)
        ))
        ;(format #t ">>>>>>>>>>> line being processed ~a\n" cs-line)
      (let ((port-location (get-source-location port
              (if (eof-object? cs-line)
                0
                (string-contains initial-line cs-line)))))
        (if (eof-object? cs-line)
          '*eoi*
          (let ((result (tokeniz cs-line port-location)))
            ; This is a sanity check for the tokenizer
            (if (pair? result)
              (begin
                (set! cs-line (cdr result))
                ;(format #t "~a " (lexical-token-category (car result)))
                (car result))
              (error (format #f "Tokenizer issue => ~a," result))
            )))))

  )
)

(define cs-parser
  (lalr-parser
    ;; Options mainly for debugging
    ;; output a parser, called ghost-parser, in a separate file - ghost.yy.scm,
    ;(output: ghost-parser "ghost.yy.scm")
    ;;; output the LALR table to ghost.out
    ;(out-table: "ghost.out")
    ;; there should be no conflict
    ;(expect:    5)

    ; Tokens (aka terminal symbol) definition
    ; https://www.gnu.org/software/bison/manual/bison.html#How-Precedence
    ; (left: token-x) reduces on token-x, while (right: token-x) shifts on
    ; token-x.
    ;
    ; NOTE
    ; LSBRACKET RSBRACKET = Square Brackets []
    ; LPAREN RPAREN = parentheses ()
    ; DQUOTE = Double quote "
    ; ID = Identifier or Marking
    ; LITERAL~COMMAND = Dictionary Keyword Sets
    ; *~n = Range-restricted Wildcards
    ; MVAR = Match Variables
    ; ? = Comparison tests
    (CONCEPT TOPIC RESPONDERS REJOINDERS GAMBIT COMMENT SAMPLE_INPUT WHITESPACE
      COMMA
      (right: LPAREN LSBRACKET << ID VAR * ^ < LITERAL NUM LITERAL~COMMAND
              *~n MVAR)
      (left: RPAREN RSBRACKET >> > DQUOTE NOT)
      (right: ? CR NEWLINE)
    )

    ; Parsing rules (aka nonterminal symbols)
    (lines
      (line) : (if $1 (format #t "\nline is ~a\n" $1))
      (lines line) : (if $2 (format #t "\nlines: ~a\n" $2))
    )

    (line
      (declarations) : $1
      (rules) : $1
      (enter) : $1
      (COMMENT) : #f
      (SAMPLE_INPUT) : #f ; TODO replace with a tester function
    )

    (declarations
      (CONCEPT ID a-sequence) :
        (display-token (format #f "concept(~a = ~a)" $2 $3))
      (TOPIC ID a-sequence) :
        (display-token (format #f "topic(~a = ~a)" $2 $3))
    )

    (rules
      (RESPONDERS a-literal a-sequence patterns) :
        (display-token (format #f "responder_~a(~a -> ~a)" $2 $3 $4))
      ; Unlabeled responder.
      ; TODO: Maybe should be labeled internally in the atomspace???
      (RESPONDERS a-sequence patterns) :
        (display-token (format #f "responder_~a(~a -> ~a)" $1 $2 $3))
      (REJOINDERS a-sequence patterns) :
        (display-token (format #f "rejoinder_~a(~a -> ~a)" $1 $2 $3))
      (GAMBIT patterns) : (display-token (format #f "gambit(~a)" $2))
    )

    (patterns
      (enter) :  $1
      (pattern) : (display-token $1)
      (patterns pattern) : (display-token (format #f "~a ~a" $1 $2))
      (patterns enter) : (display-token (format #f "~a" $1))
    )

    (enter
      (an-enter) : $1
      (enter an-enter) : $1
    )

    (an-enter
      (CR) : $1
      (NEWLINE) : #f
    )

    ; TODO: Give this a better name. Maybe should be divided into
    ; action-pattern and context-pattern ????
    (pattern
      (a-literal) : (display-token $1)
      (a-literal ?) : (display-token (format #f "~a~a" $1 $2))
      (variable)  : (display-token $1)
      (collections) : (display-token $1)
      (unordered-matching) : (display-token $1)
      (function) : (display-token $1)
      (NOT pattern) : (display-token (format #f "Not(~a)" $2))
      (< patterns) : (display-token (format #f "restart_matching(~a)" $2))
      ; FIXME end_of_sentence is not necessarily complete.
      (pattern >) : (display-token (format #f "@end_of_sentence(~a)" $1))
      (a-sequence) : (display-token $1)
      (phrase) : (display-token $1)
      (variable ? collections) :
        (display-token (format #f "is_member(~a ~a)" $1 $3))
    )

    (collections
      (choice) : (display-token $1)
      (concept) : (display-token $1)
    )

    (choice
      (LSBRACKET patterns RSBRACKET) :
        (display-token (format #f "choices(~a)" $2))
    )

    (concept
      (ID) : (display-token (format #f "concept(~a)" $1))
    )

    ; TODO: This has a restart_matching effect. See chatscript documentation
    (unordered-matching
      (<< patterns >>) : (display-token (format #f "unordered-matching(~a)" $2))
    )

    (function
      (^ a-literal LPAREN args RPAREN) :
        (display-token (format #f "function_~a(~a)" $2 $4))
      (^ a-literal LPAREN RPAREN) :
        (display-token (format #f "function_~a()" $2))
    )

    (args
      (arg) : (display-token $1)
      (args arg) :  (display-token (format #f "~a ~a" $1 $2))
    )

    (arg
      (LITERAL) :  (display-token $1)
      (concept) :  (display-token $1)
    )

    (a-sequence
      (LPAREN patterns RPAREN) : (display-token (format #f "sequence(~a)" $2))
    )

    (phrase
      (DQUOTE patterns DQUOTE) : (display-token (format #f "phrase(~a)" $2))
    )

    (variable
      (VAR LITERAL) : (display-token (format #f "variable(~a)" $2))
      (VAR collections) : (display-token (format #f "variable(~a)" $2))
      (MVAR) : (display-token (format #f "match_variable(~a)" $1))
    )

    (a-literal
      (LITERAL) : (display-token $1)
      (*~n) : (display-token (format #f "range_restricted(* ~a)" $1))
      (*) : (display-token $1)
      (LITERAL~COMMAND) :
        (display-token (format #f "command(~a -> ~a)" (car $1) (cdr $1)))
    )
  )
)

; Test lexer
(define (test-lexer lexer)
"
  For testing the lexer pass a lexer created by make-cs-lexer.
"
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

(define (main args)
"
  This function is to be used from the command-line as follows

  guile -e main -s parse.scm -f path/to/cs/file.top
"
  (let* ((option-spec
            '((file (single-char #\f) (required? #t) (value #t))))
  ;(predicate file-exists?)))) FIXME: Wrong type to apply: file-exists?
    (options (getopt-long args option-spec))
    (input-file (option-ref options 'file #f)))
    (if input-file
      (begin
        (format #t "\n--------- Starting parsing of ~a ---------\n" input-file)
        (cs-parser (make-cs-lexer input-file) error)
        (format #t "\n--------- Finished parsing of ~a ---------\n" input-file)
      )))
)

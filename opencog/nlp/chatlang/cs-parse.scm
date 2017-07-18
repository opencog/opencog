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
    ((has-match? "^[ ]*'_[0-9]" str)
      (result:suffix 'MOVAR location
        (substring (string-trim (match:substring current-match)) 2)))
    ((has-match? "^[ ]*_" str) (result:suffix 'VAR location #f))
    ; For dictionary keyword sets
    ((has-match? "^[ ]*[a-zA-Z]+~[a-zA-Z1-9]+" str)
      (result:suffix 'LEMMA~COMMAND location (command-pair)))
    ; Range-restricted Wildcards.
    ; TODO Maybe replace with dictionary keyword sets then process it on action?
    ((has-match? "^[ ]*\\*~[0-9]+" str)
      (result:suffix '*~n location
        (substring (string-trim (match:substring current-match)) 2)))
    ((has-match? "^[ ]*~[a-zA-Z_]+" str)
      (result:suffix 'ID location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*\\^" str) (result:suffix '^ location #f))
    ((has-match? "^[ ]*\\[" str) (result:suffix 'LSBRACKET location #f))
    ((has-match? "^[ ]*]" str) (result:suffix 'RSBRACKET location #f))
    ((has-match? "^[ ]*<<" str) (result:suffix '<< location #f))
    ((has-match? "^[ ]*>>" str) (result:suffix '>> location #f))
    ; For restarting matching position -- "< *"
    ((has-match? "^[ ]*<[ ]*\\*" str) (result:suffix 'RESTART location #f))
    ; This should follow <<
    ((has-match? "^[ ]*<" str) (result:suffix '< location #f))
    ; This should follow >>
    ((has-match? "^[ ]*>" str) (result:suffix '> location #f))
    ((has-match? "^[ ]*\"" str) (result:suffix 'DQUOTE location "\""))
    ; Precise wildcards
    ((has-match? "^[ ]*\\*[0-9]+" str) (result:suffix '*n location
       (substring (string-trim (match:substring current-match)) 1)))
    ; Wildcards
    ((has-match? "^[ ]*\\*" str) (result:suffix '* location "*"))
    ((has-match? "^[ ]*!" str) (result:suffix 'NOT location #f))
    ((has-match? "^[ ]*[?]" str) (result:suffix '? location "?"))
    ; Literals -- words start with a '
    ((has-match? "^[ ]*'[a-zA-Z]+\\b" str)
      (result:suffix 'LITERAL location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*[a-zA-Z'-]+\\b" str)
      (if (is-lemma? (string-trim (match:substring current-match)))
        (result:suffix 'LEMMA location
          (string-trim (match:substring current-match)))
        ; Literals, words in the pattern that are not in their canonical forms
        (result:suffix 'LITERAL location
          (string-trim (match:substring current-match)))))
    ; This should always be near the end, because it is broadest of all.
    ((has-match? "^[ \t]*['.,_!?0-9a-zA-Z-]+" str)
        (result:suffix 'STRING location
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
                ; For debugging
                ;(format #t "=== tokeniz: ~a\n-> ~a\n"
                ;  cs-line (lexical-token-category (car result)))
                (set! cs-line (cdr result))
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
    ; LEMMA~COMMAND = Dictionary Keyword Sets
    ; *~n = Range-restricted Wildcards
    ; MVAR = Match Variables
    ; MOVAR = Match Variables grounded in their original words
    ; ? = Comparison tests
    (CONCEPT TOPIC RESPONDERS REJOINDERS GAMBIT COMMENT SAMPLE_INPUT WHITESPACE
      (right: LPAREN LSBRACKET << ID VAR * ^ < LEMMA LITERAL NUM LEMMA~COMMAND
              STRING *~n *n MVAR MOVAR NOT RESTART)
      (left: RPAREN RSBRACKET >> > DQUOTE)
      (right: ? CR NEWLINE)
    )

    ; Parsing rules (aka nonterminal symbols)
    (inputs
      (input) : (if $1 (format #t "\nInput: ~a\n" $1))
      (inputs input) : (if $2 (format #t "\nInput: ~a\n" $2))
    )

    (input
      (declarations) : $1
      (rules) : $1
      (enter) : $1
      (COMMENT) : #f
      (SAMPLE_INPUT) : #f ; TODO replace with a tester function
    )

    (enter
      (an-enter) : $1
      (enter an-enter) : $1
    )

    (an-enter
      (CR) : $1
      (NEWLINE) : #f
    )

    ; Declaration/annotation(for ghost) grammar
    (declarations
      (CONCEPT ID declaration-sequence) :
        (display-token (format #f "concept(~a = ~a)" $2 $3))
      (TOPIC ID declaration-sequence) :
        (display-token (format #f "topic(~a = ~a)" $2 $3))
    )

    (declaration-sequence
      (LPAREN declaration-members RPAREN) :
        (display-token (format #f "declaration-sequence(~a)" $2))
    )

    (declaration-members
      (declaration-member) : $1
      (declaration-members declaration-member) :
        (display-token (format #f "~a ~a" $1 $2))
    )

    (declaration-member
      (LEMMA) :  $1
      (LITERAL) : $1
      (STRING) : $1
      (concept) :  $1
      (LEMMA~COMMAND) :
        (display-token (format #f "command(~a -> ~a)" (car $1) (cdr $1)))
    )

    ; Rule grammar
    (rules
      (RESPONDERS word context-sequence action-patterns) :
        (display-token (format #f "responder_~a label_~a (~a -> ~a)"
          $1 $2 $3 $4))
      ; Unlabeled responder.
      ; TODO: Maybe should be labeled internally in the atomspace???
      (RESPONDERS context-sequence action-patterns) :
        (display-token (format #f "responder_~a(~a -> ~a)" $1 $2 $3))
      (REJOINDERS context-sequence action-patterns) :
        (display-token (format #f "rejoinder_~a(~a -> ~a)" $1 $2 $3))
      (GAMBIT action-patterns) : (display-token (format #f "gambit(~a)" $2))
    )

    (context-sequence
      (LPAREN context-patterns RPAREN) :
        ; Rearrange the terms in the intended order if there are any
        ; sentence anchors in the context, and append those implicit
        ; wildcards to the beginning and/or the end of the pattern
        ; if appropriate
        (format #f "context(~a)"
          (let* ((wc "wildcard(0 -1)")
                 (start-anchor (string-match "start_of_sentence\\(\\)" $2))
                 (end-anchor (string-match "end_of_sentence\\(\\)" $2)))
            (cond ; e.g. turning "< A B C >" into "* A B C *"
                  ((and start-anchor end-anchor)
                   (string-trim-both (substring $2
                     (match:end start-anchor) (match:start end-anchor))))
                  ; e.g. turning "< A B C" into "A B C *"
                  ((and start-anchor (string-null? (match:prefix start-anchor)))
                   (string-append (string-trim-both
                     (match:suffix start-anchor)) " " wc))
                  ; e.g. turning "C < A B" into "A B * C *"
                  (start-anchor (string-append (string-trim-both
                    (match:suffix start-anchor)) " " wc " "
                      (string-trim-both (match:prefix start-anchor)) " " wc))
                  ; e.g. turning "A B C >" into "* A B C"
                  ((and end-anchor (string-null? (match:suffix end-anchor)))
                   (string-append wc " " (string-trim-both
                     (match:prefix end-anchor))))
                  ; e.g. turning "C > A B" into "* A B * C"
                  (end-anchor (string-append wc " " (string-trim-both
                    (match:suffix end-anchor)) " " wc " "
                      (string-trim-both (match:prefix end-anchor))))
                  ; if there is no sentence anchor at all, append those implicit
                  ; wildcards at the beginning and the end of the pattern
                  ; e.g. turning "A B C" into "* A B C *"
                  (else (string-append wc " " $2 " " wc))))
        )
    )

    (context-patterns
      (context-pattern) : (display-token $1)
      (context-patterns context-pattern) :
        (display-token (format #f "~a ~a" $1 $2))
      (context-patterns enter) : $1
    )

    (context-pattern
      (<) : (display-token "start_of_sentence()")
      (>) : (display-token "end_of_sentence()")
      (*) : (display-token (format #f "wildcard(0 -1)"))
      (*n) : (display-token (format #f "wildcard(~a ~a)" $1 $1))
      (*~n) : (display-token (format #f "wildcard(0 ~a)" $1))
      (LEMMA) : (display-token (format #f "lemma(~a)" $1))
      (LITERAL) : (display-token (format #f "literal(~a)" $1))
      (phrase) : (display-token $1)
      (concept) : (display-token $1)
      (variable) : (display-token $1)
      (function) : (display-token $1)
      (choice) : (display-token $1)
      (unordered-matching) : (display-token $1)
      (NOT LEMMA) : (display-token (format #f "not(lemma(~a))" $2))
      (NOT LITERAL) : (display-token (format #f "not(literal(~a))" $2))
      (NOT concept) : (display-token (format #f "not(~a)" $2))
      (NOT choice) : (display-token (format #f "not(~a)" $2))
      (variable ? concept) :
        (display-token (format #f "is_member(~a ~a)" $1 $3))
      (sequence) : (display-token $1)
    )

    (action-patterns
      (action-pattern) : (display-token $1)
      (action-patterns action-pattern) :
        (display-token (format #f "~a ~a" $1 $2))
      (action-patterns enter) : (display-token $1)
    )

    (action-pattern
      (?) : (display-token "?")
      (NOT) : (display-token "!")
      (LEMMA) : (display-token $1)
      (LITERAL) : (display-token $1)
      (STRING) : (display-token $1)
      (DQUOTE words DQUOTE) : (display-token (format #f "~a ~a ~a" $1 $2 $3))
      (variable) : (display-token $1)
      (function) : (display-token $1)
      (LSBRACKET action-patterns RSBRACKET) :
        (display-token (format #f "action-choice(~a)" $2))
    )

    (sequence
      (LPAREN sequence-terms RPAREN) : (format #f "sequence(~a)" $2)
    )

    (sequence-terms
      (sequence-term) : $1
      (sequence-terms sequence-term) : (format #f "~a ~a" $1 $2)
    )

    (sequence-term
      (*) : (display-token (format #f "wildcard(0 -1)"))
      (*n) : (display-token (format #f "wildcard(~a ~a)" $1 $1))
      (*~n) : (display-token (format #f "wildcard(0 ~a)" $1))
      (LEMMA) : (display-token (format #f "lemma(~a)" $1))
      (LITERAL) : (display-token (format #f "literal(~a)" $1))
      (phrase) : (display-token $1)
      (concept) : (display-token $1)
      (variable) : (display-token $1)
      (choice) : (display-token $1)
    )

    (choice
      (LSBRACKET context-patterns RSBRACKET) :
        (display-token (format #f "choices(~a)" $2))
    )

    (concept
      (ID) : (display-token (format #f "concept(~a)" $1))
    )

    ; TODO: This has a restart_matching effect. See chatscript documentation
    (unordered-matching
      (<< unordered-terms >>) :
        (display-token (format #f "unordered-matching(~a)" $2))
      ; Couldn't come up with any better and robust way than stacking up
      ; RESTART tokens like this...
      (unordered-term RESTART unordered-term) :
        (display-token (format #f "unordered-matching(~a ~a)" $1 $3))
      (unordered-term RESTART unordered-term RESTART unordered-term) :
        (display-token (format #f "unordered-matching(~a ~a ~a)" $1 $3 $5))
      (unordered-term RESTART unordered-term RESTART unordered-term
        RESTART unordered-term) :
          (display-token (format #f "unordered-matching(~a ~a ~a ~a)"
            $1 $3 $5 $7))
      (unordered-term RESTART unordered-term RESTART unordered-term
        RESTART unordered-term RESTART unordered-term) :
          (display-token (format #f "unordered-matching(~a ~a ~a ~a ~a)"
            $1 $3 $5 $7 $9))
    )

    (unordered-terms
      (unordered-term) : (display-token $1)
      (unordered-terms unordered-term) : (display-token (format #f "~a ~a" $1 $2))
    )

    (unordered-term
      (LEMMA) : (display-token (format #f "lemma(~a)" $1))
      (LITERAL) : (display-token (format #f "literal(~a)" $1))
      (phrase) : (display-token $1)
      (concept) : (display-token $1)
      (choice) : (display-token $1)
    )

    (function
      (^ word LPAREN args RPAREN) :
        (display-token (format #f "function_~a(~a)" $2 $4))
      (^ word LPAREN RPAREN) :
        (display-token (format #f "function_~a()" $2))
      (^ word) :
        (display-token (format #f "function_~a" $2))
    )

    (args
      (arg) : (display-token $1)
      (args arg) : (display-token (format #f "~a ~a" $1 $2))
    )

    (arg
      (LEMMA) :  (display-token (format #f "lemma(~a)" $1))
      (LITERAL) :  (display-token (format #f "literal(~a)" $1))
      (concept) :  (display-token $1)
      (variable) : (display-token $1)
    )

    (phrase
      (DQUOTE words DQUOTE) : (display-token (format #f "phrase(~a)" $2))
    )

    (words
      (word) : $1
      (words word) : $1
    )

    (word
      (LEMMA) : $1
      (LITERAL) : $1
      (STRING) : $1
    )

    (variable
      (VAR *) : (display-token "variable(wildcard(0 -1))")
      (VAR *n) : (display-token (format #f "variable(wildcard(~a ~a))" $2 $2))
      (VAR *~n) : (display-token (format #f "variable(wildcard(0 ~a))" $2))
      (VAR LEMMA) :  (display-token (format #f "variable(lemma(~a))" $2))
      (VAR concept) : (display-token (format #f "variable(~a)" $2))
      (VAR choice) : (display-token (format #f "variable(~a)" $2))
      (MVAR) : (display-token (format #f "match_variable(~a)" $1))
      (MOVAR) : (display-token (format #f "match_orig_variable(~a)" $1))
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

(define-public (test-parse line)
"
  Parse a text string in a Guile shell, for debugging mainly.
"
  (cs-parser (cs-lexer (open-input-string line)) error)
)

(define-public (test-parse-file file)
"
  Parse a topic file in a Guile shell, for debugging mainly.
"
  (cs-parser (cs-lexer (open-file-input-port file)) error)
)

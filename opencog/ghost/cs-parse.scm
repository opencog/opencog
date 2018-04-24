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
    ((has-match? "^[ \t]*urge:" str) (result:suffix 'URGE location #f))
    ((has-match? "^[ \t]*ordered-goal:" str) (result:suffix 'ORD-GOAL location #f))
    ((has-match? "^[ \t]*goal:" str) (result:suffix 'GOAL location #f))
    ((has-match? "^[ \t]*#goal:" str) (result:suffix 'RGOAL location #f))
    ((has-match? "^[ \t]*#!" str) ; This should be checked always before #
      ; TODO Add tester function for this
      (cons (make-lexical-token 'SAMPLE_INPUT location #f) ""))
    ((has-match? "^[ \t]*#" str)
      (cons (make-lexical-token 'COMMENT location #f) ""))
    ; Chatscript rules
    ((has-match? "^[ ]*[s?u]:" str)
      (result:suffix 'RESPONDERS location
        (car (string->list (substring (string-trim
          (match:substring current-match)) 0 1)))))
    ((has-match? "^[ \t]*[a-q]:" str)
      (result:suffix 'REJOINDERS location
        (car (string->list (substring (string-trim
          (match:substring current-match)) 0 1)))))
    ((has-match? "^[ ]*[rt]:" str)
      (result:suffix 'GAMBIT location
        (car (string->list (substring (string-trim
          (match:substring current-match)) 0 1)))))
    ((has-match? "^[ ]*_[0-9]" str)
      (result:suffix 'MVAR location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*'_[0-9]" str)
      (result:suffix 'MOVAR location
        (substring (string-trim (match:substring current-match)) 2)))
    ((has-match? "^[ ]*\\$[a-zA-Z0-9_-]+" str)
      (result:suffix 'UVAR location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*_" str) (result:suffix 'VAR location #f))
    ; For dictionary keyword sets
    ((has-match? "^[ ]*[a-zA-Z]+~[a-zA-Z1-9]+" str)
      (result:suffix 'DICTKEY location (command-pair)))
    ; Range-restricted Wildcards.
    ; TODO Maybe replace with dictionary keyword sets then process it on action?
    ((has-match? "^[ ]*\\*~[0-9]+" str)
      (result:suffix '*~n location
        (substring (string-trim (match:substring current-match)) 2)))
    ((has-match? "^[ ]*~[a-zA-Z0-9_]+" str)
      (result:suffix 'ID location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*\\^" str) (result:suffix '^ location #f))
    ((has-match? "^[ ]*\\[" str) (result:suffix 'LSBRACKET location #f))
    ((has-match? "^[ ]*]" str) (result:suffix 'RSBRACKET location #f))
    ((has-match? "^[ ]*\\{" str) (result:suffix 'LBRACE location #f))
    ((has-match? "^[ ]*}" str) (result:suffix 'RBRACE location #f))
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
    ((has-match? "^[ ]*=" str) (result:suffix 'EQUAL location #f))
    ; Literals -- words start with a '
    ((has-match? "^[ ]*'[a-zA-Z]+\\b" str)
      (result:suffix 'LITERAL location
        (substring (string-trim (match:substring current-match)) 1)))
    ((has-match? "^[ ]*[a-zA-Z’'-]+\\b" str)
      (if (is-lemma? (string-trim (match:substring current-match)))
        (result:suffix 'LEMMA location
          (string-trim (match:substring current-match)))
        ; Literals, words in the pattern that are not in their canonical forms
        (result:suffix 'LITERAL location
          (string-trim (match:substring current-match)))))
    ((has-match? "^[ ]*[0-9]+[0-9.]*" str)
      (result:suffix 'NUM location
        (string-trim (match:substring current-match))))
    ((has-match? "^[ ]*[|]" str)
     (result:suffix 'VLINE location
        (string-trim (match:substring current-match))))
    ((has-match? "^[ ]*," str)
     (result:suffix 'COMMA location
        (string-trim (match:substring current-match))))
    ; This should always be near the end, because it is broadest of all.
    ((has-match? "^[ \t]*[~’'._!?0-9a-zA-Z-]+" str)
        (result:suffix 'STRING location
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
          (cog-logger-debug ghost-logger
            "\n-------------------------- line ~a \n" (port-line port))
          (set! cs-line (read-line port))
          (set! initial-line cs-line)
        ))
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
                (cog-logger-debug ghost-logger "=== tokeniz: ~a\n-> ~a\n"
                  cs-line (lexical-token-category (car result)))
                (set! cs-line (cdr result))
                (car result))
              (error (format #f "Tokenizer issue => ~a," result))
            )))))
  )
)

(define (cs-parser)
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
    ; LBRACE RBRACE = Braces {}
    ; DQUOTE = Double quote "
    ; ID = Identifier or Marking
    ; DICTKEY = Dictionary Keyword Sets
    ; *~n = Range-restricted Wildcards
    ; MVAR = Match Variables
    ; MOVAR = Match Variables grounded in their original words
    ; ? = Comparison tests
    ; VLINE = Vertical Line |
    (CONCEPT TOPIC RESPONDERS REJOINDERS GAMBIT URGE ORD-GOAL GOAL RGOAL COMMENT
     SAMPLE_INPUT WHITESPACE
      (right: LPAREN LSBRACKET << ID VAR * ^ < LEMMA LITERAL NUM DICTKEY
              STRING *~n *n UVAR MVAR MOVAR EQUAL NOT RESTART LBRACE VLINE COMMA)
      (left: RPAREN RSBRACKET RBRACE >> > DQUOTE)
      (right: ? CR NEWLINE)
    )

    ; Parsing rules (aka nonterminal symbols)
    (inputs
      (input) : #t
      (inputs input) : #t
    )

    (input
      (declarations) : $1
      (urge) : (set-initial-urge (eval-string (string-append "(list " $1 ")")))
      (goal) : (create-top-lv-goal (eval-string (string-append "(list " $1 ")")))
      (ordered-goal) :
        (create-top-lv-goal (eval-string (string-append "(list " $1 ")")) #t)
      (rule) : $1
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
        (create-concept $2 (eval-string (string-append "(list " $3 ")")))
      (TOPIC ID declaration-sequence) :
        (create-topic $2 (list) (eval-string (string-append "(list " $3 ")")))
      (TOPIC ID LPAREN RPAREN) : (create-topic $2 (list) (list))
      (TOPIC ID LSBRACKET RSBRACKET) : (create-topic $2 (list) (list))
      ; Note, "names" here are the topic-level features/functions that will
      ; be applied to every single rules under this topic
      (TOPIC ID names declaration-sequence) :
        (create-topic $2 (string-split $3 #\sp)
          (eval-string (string-append "(list " $4 ")")))
      (TOPIC ID names LPAREN RPAREN) :
        (create-topic $2 (string-split $3 #\sp) (list))
      (TOPIC ID names LSBRACKET RSBRACKET) :
        (create-topic $2 (string-split $3 #\sp) (list))
      (UVAR EQUAL name) : (create-user-variable $1 $3)
    )

    (declaration-sequence
      (LPAREN declaration-members RPAREN) : $2
      (LSBRACKET declaration-members RSBRACKET) : $2
    )

    (declaration-members
      (declaration-member) : $1
      (declaration-members declaration-member) : (format #f "~a ~a" $1 $2)
    )

    (declaration-member
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
      (concept) : $1
      (sequence) : $1
      (DICTKEY) :
        (format #f "(cons 'dictkey (cons \"~a\" \"~a\"))" (car $1) (cdr $1))
    )

    ; Rule grammar
    (rule
      (rule-goal RESPONDERS name context action) :
        (create-rule
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $5 ")"))
          (eval-string (string-append "(list " $1 ")"))
          $3 $2)
      (rule-goal RESPONDERS context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $1 ")"))
          "" $2)
      (RESPONDERS name context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (list) $2 $1)
      (RESPONDERS context action) :
        (create-rule
          (eval-string (string-append "(list " $2 ")"))
          (eval-string (string-append "(list " $3 ")"))
          (list) "" $1)
      (rule-goal REJOINDERS name context action) :
        (create-rule
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $5 ")"))
          (eval-string (string-append "(list " $1 ")"))
          $3 $2)
      (rule-goal REJOINDERS context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $1 ")"))
          "" $2)
      ; TODO: Make sure there is a parent rule before
      ; accepting any rejoinders
      (REJOINDERS name context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (list) $2 $1)
      (REJOINDERS context action) :
        (create-rule
          (eval-string (string-append "(list " $2 ")"))
          (eval-string (string-append "(list " $3 ")"))
          (list) "" $1)
      ; Note, do not support a gambit that has a "name"
      ; but no context -- it's ambiguous to determine
      ; whether it's really a "name" or just the first
      ; word of the "action"
      (rule-goal GAMBIT name context action) :
        (create-rule
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $5 ")"))
          (eval-string (string-append "(list " $1 ")"))
          $3 $2)
      (rule-goal GAMBIT context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (eval-string (string-append "(list " $1 ")"))
          "" $2)
      (rule-goal GAMBIT action) :
        (create-rule
          (list)
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $1 ")"))
          "" $2)
      ; Same as the above, context is needed for a
      ; named gambit
      (GAMBIT name context action) :
        (create-rule
          (eval-string (string-append "(list " $3 ")"))
          (eval-string (string-append "(list " $4 ")"))
          (list) $2 $1)
      (GAMBIT context action) :
        (create-rule
          (eval-string (string-append "(list " $2 ")"))
          (eval-string (string-append "(list " $3 ")"))
          (list) "" $1)
      (GAMBIT action) :
        (create-rule
          (list)
          (eval-string (string-append "(list " $2 ")"))
          (list) "" $1)
    )

    (urge
      (URGE LPAREN goal-members RPAREN) : $3
    )

    (ordered-goal
      (ORD-GOAL LPAREN goal-members RPAREN) : $3
    )

    (rule-goal
      (RGOAL LPAREN goal-members RPAREN) : $3
    )

    (goal
      (GOAL LPAREN goal-members RPAREN) : $3
    )

    (goal-members
      (goal-member) : $1
      (goal-members goal-member) : (format #f "~a ~a" $1 $2)
    )

    (goal-member
      (LITERAL EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
      (LEMMA EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
      (STRING EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
    )

    (context
      (LPAREN RPAREN) : "(cons 'wildcard (cons 0 -1))"
      (LPAREN negation RPAREN) : $2
      (LPAREN context-patterns RPAREN) : $2
      (LPAREN negation context-patterns RPAREN) : (format #f "~a ~a" $2 $3)
      (LPAREN unordered-matching RPAREN) : $2
      (LPAREN negation unordered-matching RPAREN) : (format #f "~a ~a" $2 $3)
    )

    (context-patterns
      (context-pattern) : $1
      (context-patterns context-pattern) : (format #f "~a\n~a" $1 $2)
      (context-patterns enter) : $1
    )

    (context-pattern
      (<) : "(cons 'anchor-start \"<\")"
      (>) : "(cons 'anchor-end \">\")"
      (wildcard) : $1
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
      (concept) : $1
      (variable) : $1
      (user-variable) : $1
      (function) : $1
      (choice) : $1
      (optional) : $1
      (variable ? concept) :
        (format #f "(cons 'is_member (list ~a ~a))" $1 $3)
      (sequence) : $1
    )

    (action
      (action-patterns) : (format #f "(cons 'action (list ~a))" $1)
    )

    (action-patterns
      (action-pattern) : $1
      (action-patterns action-pattern) : (format #f "~a ~a" $1 $2)
      (action-patterns enter) : $1
    )

    (action-pattern
      (?) : "(cons 'str \"?\")"
      (NOT) : "(cons 'str \"!\")"
      (COMMA) : "(cons 'str \",\")"
      (DQUOTE) : "(cons 'str \"\\\"\")"
      (LEMMA) : (format #f "(cons 'str \"~a\")" $1)
      (LITERAL) : (format #f "(cons 'str \"~a\")" $1)
      (NUM) : (format #f "(cons 'str \"~a\")" $1)
      (STRING) : (format #f "(cons 'str \"~a\")" $1)
      (variable) : $1
      ; e.g. $username
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
      ; e.g. $username=Bob
      (UVAR EQUAL name) :
        (format #f "(cons 'assign_uvar (list \"~a\" (cons 'str \"~a\")))" $1 $3)
      ; e.g. $username='_0
      (UVAR EQUAL variable-grounding) :
        (format #f "(cons 'assign_uvar (list \"~a\" ~a))" $1 $3)
      (function) : $1
      (tts-feature) : $1
      (LSBRACKET action-patterns RSBRACKET) :
        (format #f "(cons 'action-choices (list ~a))" $2)
    )

    (lemma
      (LEMMA) : (format #f "(cons 'lemma \"~a\")" $1)
    )

    (literal
      (LITERAL) : (format #f "(cons 'word \"~a\")" $1)
      (STRING) : (format #f "(cons 'word \"~a\")" $1)
    )

    (phrase
      (DQUOTE phrase-terms DQUOTE) : (format #f "(cons 'phrase \"~a\")" $2)
    )

    (phrase-terms
      (phrase-term) : $1
      (phrase-terms phrase-term) : (format #f "~a ~a" $1 $2)
    )

    (phrase-term
      (LEMMA) : $1
      (LITERAL) : $1
      (STRING) : $1
    )

    (concept
      (ID) : (format #f "(cons 'concept \"~a\")" $1)
    )

    (optional
      (LBRACE choice-terms RBRACE) :
        (format #f "(cons 'optionals (list ~a))" $2)
    )

    (choice
      (LSBRACKET choice-terms RSBRACKET) :
        (format #f "(cons 'choices (list ~a))" $2)
    )

    (choice-terms
      (choice-term) : $1
      (choice-terms choice-term) : (format #f "~a ~a" $1 $2)
    )

    (choice-term
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
      (concept) : $1
      (sequence) : $1
    )

    (wildcard
      (*) : "(cons 'wildcard (cons 0 -1))"
      (*n) : (format #f "(cons 'wildcard (cons ~a ~a))" $1 $1)
      (*~n) : (format #f "(cons 'wildcard (cons 0 ~a))" $1)
    )

    (variable
      (VAR wildcard) : (format #f "(cons 'variable (list ~a))" $2)
      (VAR lemma) :  (format #f "(cons 'variable (list ~a))" $2)
      (VAR concept) : (format #f "(cons 'variable (list ~a))" $2)
      (VAR choice) : (format #f "(cons 'variable (list ~a))" $2)
      (variable-grounding) : $1
    )

    (variable-grounding
      (MVAR) : (format #f "(cons 'get_lvar ~a)" $1)
      (MOVAR) : (format #f "(cons 'get_wvar ~a)" $1)
    )

    (user-variable
      (UVAR) : (format #f "(cons 'uvar_exist \"~a\")" $1)
      (UVAR EQUAL name) :
        (format #f "(cons 'uvar_equal (list \"~a\" \"~a\"))" $1 $3)
    )

    (negation
      (NOT negation-term) : (format #f "(cons 'negation (list ~a))" $2)
      (NOT LSBRACKET negation-terms RSBRACKET) :
        (format #f "(cons 'negation (list ~a))" $3)
    )

    (negation-terms
      (negation-term) : $1
      (negation-terms negation-term) : (format #f "~a ~a" $1 $2)
    )

    (negation-term
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
      (concept) : $1
    )

    (function
      (^ name LPAREN args RPAREN) :
        (format #f "(cons 'function (list \"~a\" ~a))" $2 $4)
      (^ name LPAREN RPAREN) :
        (format #f "(cons 'function (list \"~a\"))" $2)
      (^ name) :
        (format #f "(cons 'function (list \"~a\"))" $2)
    )

    (names
      (name) : $1
      (names name) : (format #f "~a ~a" $1 $2)
    )

    (name
      (LEMMA) : $1
      (LITERAL) : $1
      (STRING) : $1
      (NUM) : $1
    )

    (args
      (arg) : $1
      (arg COMMA args) : (format #f "~a ~a" $1 $3)
    )

    (arg
      (LEMMA) : (format #f "(cons 'arg \"~a\")" $1)
      (LITERAL) : (format #f "(cons 'arg \"~a\")" $1)
      (NUM) : (format #f "(cons 'arg \"~a\")" $1)
      (STRING) : (format #f "(cons 'arg \"~a\")" $1)
      (variable-grounding) : $1
    )

    (sequence
      (LPAREN sequence-terms RPAREN) :
        (format #f "(cons 'sequence (list ~a))" $2)
    )

    (sequence-terms
      (sequence-term) : $1
      (sequence-terms sequence-term) : (format #f "~a ~a" $1 $2)
    )

    (sequence-term
      (wildcard) : $1
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
    )

    ; TODO: This has a restart_matching effect. See chatscript documentation
    (unordered-matching
      (<< unordered-terms >>) :
        (format #f "(cons 'unordered-matching (list ~a))" $2)
      ; Couldn't come up with any better and robust way than stacking up
      ; RESTART tokens like this...
      (unordered-term RESTART unordered-term) :
        (format #f "(cons 'unordered-matching (list ~a ~a))" $1 $3)
      (unordered-term RESTART unordered-term RESTART unordered-term) :
        (format #f "(cons 'unordered-matching (list ~a ~a ~a))" $1 $3 $5)
      (unordered-term RESTART unordered-term RESTART unordered-term
        RESTART unordered-term) :
          (format #f "(cons 'unordered-matching (list ~a ~a ~a ~a))"
            $1 $3 $5 $7)
      (unordered-term RESTART unordered-term RESTART unordered-term
        RESTART unordered-term RESTART unordered-term) :
          (format #f "(cons 'unordered-matching (list ~a ~a ~a ~a ~a))"
            $1 $3 $5 $7 $9)
    )

    (unordered-terms
      (unordered-term) : $1
      (unordered-terms unordered-term) : (format #f "~a ~a" $1 $2)
    )

    (unordered-term
      (lemma) : $1
      (literal) : $1
      (phrase) : $1
      (concept) : $1
      (choice) : $1
      (negation) : $1
    )

    ; For things like: |pause|, |vocal,27|, and |worry,$med,3.0| etc
    (tts-feature
      (VLINE tts-members VLINE) :
        (format #f "(cons 'tts-feature (list ~a))" $2)
    )

    (tts-members
      (tts-member) : $1
      (tts-members tts-member) : (format #f "~a ~a" $1 $2)
    )

    (tts-member
      (COMMA) : ""
      (name) : (format #f "(cons 'str \"~a\")" $1)
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
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

(define-public (test-parse line)
"
  Parse a text string in a Guile shell, for debugging mainly.
"
  (define parser (cs-parser))
  (parser (cs-lexer (open-input-string line)) error)
)

(define-public (test-parse-file file)
"
  Parse a topic file in a Guile shell, for debugging mainly.
"
  (define parser (cs-parser))
  (define fp (open-file-input-port file))
  (set-port-encoding! fp "UTF-8")
  (parser (cs-lexer fp) error)
)

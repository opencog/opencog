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
    (let ((match
            (string-match
              (string-append "^[ \t]*" pattern "[ \t]*") str)))
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
      (cons (match:suffix match) (string-trim-both (match:prefix match)))
    ))

  ; NOTE
  ; 1. The matching must be done starting from the most specific to the
  ;    the broadest regex patterns.
  ; 2. #f is given as a token value for those cases which don't have any
  ;    semantic values.
  (cond
    ((has-match? "\\(" str) (result:suffix 'LPAREN location #f))
    ((has-match? "\\)" str) (result:suffix 'RPAREN location #f))
    ; Chatscript declarations
    ((has-match? "concept:" str) (result:suffix 'CONCEPT location #f))
    ; The token value for 'CR is empty-string so as to handle multiline
    ; rules.
    ((has-match? "\r" str) (result:suffix 'CR location ""))
    ; FIXME: This is not really newline.
    ((string=? "" str) (cons (make-lexical-token 'NEWLINE location #f) ""))
    ((has-match? "urge:" str) (result:suffix 'URGE location #f))
    ((has-match? "ordered-goal:" str) (result:suffix 'ORD-GOAL location #f))
    ((has-match? "goal:" str) (result:suffix 'GOAL location #f))
    ((has-match? "#goal:" str) (result:suffix 'RGOAL location #f))
    ((has-match? "parallel-rules:" str)
      (result:suffix 'PARALLEL-RULES location #f))
    ((has-match? "link-concept:" str)
      (result:suffix 'LINK-CONCEPT location #f))
    ((has-match? "#link-concept:" str)
      (result:suffix 'RLINK-CONCEPT location #f))
    ((has-match? "global-default-rule:" str)
      (result:suffix 'GLOBAL-DEFAULT-RULE location #f))
    ((has-match? "#!" str) ; This should be checked always before #
      ; TODO Add tester function for this
      (cons (make-lexical-token 'SAMPLE_INPUT location #f) ""))
    ((has-match? "*#" str)
      (cons (make-lexical-token 'COMMENT location #f) ""))
    ; Reactive rules (aka Responders)
    ; Starts with "r:"; "s:", "?:" and "u:" are for backward compatibility
    ((has-match? "[rs?u]:" str)
      (result:suffix 'REACTIVE-RULE location
        (string-trim-both
          (match:substring current-match)
            (lambda (c) (or (eqv? c #\tab) (eqv? c #\space) (eqv? c #\:))))))
    ; Rejoinders
    ; Starts with "j" follow by a number to denote the level
    ((has-match? "j[0-9]+:" str)
      (result:suffix 'REJOINDER location
        (string-trim-both
          (match:substring current-match)
            (lambda (c) (or (eqv? c #\tab) (eqv? c #\space) (eqv? c #\:))))))
    ; Rejoinders again, for backward compatibility, but limit the support
    ; up to the letter "e" instead of "q" as in ChatScript
    ((has-match? "[a-e]:" str)
      (result:suffix 'REJOINDER location
        (string-trim-both
          (match:substring current-match)
            (lambda (c) (or (eqv? c #\tab) (eqv? c #\space) (eqv? c #\:))))))
    ; Proactive rules (aka Gambits)
    ; Starts with "p:"; "t:" is for backward compatibility
    ((has-match? "[pt]:" str)
      (result:suffix 'PROACTIVE-RULE location
        (string-trim-both
          (match:substring current-match)
            (lambda (c) (or (eqv? c #\tab) (eqv? c #\space) (eqv? c #\:))))))
    ((has-match? "[{][%] set delay=[0-9]+ [%][}]" str)
      (result:suffix 'SET_DELAY location
        (string-trim-both (match:substring current-match))))
    ((has-match? "_[0-9]" str)
      (result:suffix 'MVAR location
        (substring (string-trim-both (match:substring current-match)) 1)))
    ((has-match? "'_[0-9]" str)
      (result:suffix 'MOVAR location
        (substring (string-trim-both (match:substring current-match)) 2)))
    ((has-match? "\\$[a-zA-Z0-9_-]+" str)
      (result:suffix 'UVAR location
        (substring (string-trim-both (match:substring current-match)) 1)))
    ((has-match? "_" str) (result:suffix 'VAR location #f))
    ; For dictionary keyword sets
    ((has-match? "[a-zA-Z]+~[a-zA-Z1-9]+" str)
      (result:suffix 'DICTKEY location (command-pair)))
    ; Range-restricted Wildcards.
    ; TODO Maybe replace with dictionary keyword sets then process it on action?
    ((has-match? "\\*~[0-9]+" str)
      (result:suffix '*~n location
        (substring (string-trim-both (match:substring current-match)) 2)))
    ((has-match? "~[a-zA-Z0-9_]+" str)
      (result:suffix 'ID location
        (substring (string-trim-both (match:substring current-match)) 1)))
    ((has-match? "\\^" str) (result:suffix '^ location #f))
    ((has-match? "\\[" str) (result:suffix 'LSBRACKET location #f))
    ((has-match? "]" str) (result:suffix 'RSBRACKET location #f))
    ((has-match? "\\{" str) (result:suffix 'LBRACE location #f))
    ((has-match? "}" str) (result:suffix 'RBRACE location #f))
    ((has-match? "<<" str) (result:suffix '<< location #f))
    ((has-match? ">>" str) (result:suffix '>> location #f))
    ; For restarting matching position -- "< *"
    ((has-match? "<[ ]*\\*" str) (result:suffix 'RESTART location #f))
    ; This should follow <<
    ((has-match? "<" str) (result:suffix '< location #f))
    ; This should follow >>
    ((has-match? ">" str) (result:suffix '> location #f))
    ((has-match? "\"" str) (result:suffix 'DQUOTE location "\""))
    ; Precise wildcards
    ((has-match? "\\*[0-9]+" str) (result:suffix '*n location
       (substring (string-trim-both (match:substring current-match)) 1)))
    ; Wildcards
    ((has-match? "\\*" str) (result:suffix '* location "*"))
    ((has-match? "!" str) (result:suffix 'NOT location #f))
    ((has-match? "[?]" str) (result:suffix '? location "?"))
    ((has-match? "=" str) (result:suffix 'EQUAL location #f))
    ; For time -- a.m. and p.m.
    ; Just catch it to avoid it being splitted into multiple words
    ; Should be done before any literal / lemma matching
    ((has-match? "[ap]\\.m\\." str)
      (result:suffix 'STRING location
        (string-trim-both (match:substring current-match))))
    ; Words with apostrophe, e.g. I'm, it's etc
    ((has-match? "[a-zA-Z]+['’][a-zA-Z]+" str)
      (result:suffix 'LITERAL_APOS location
        (string-trim-both (match:substring current-match))))
    ; Literals for example: Mr. Dr. etc
    ((has-match? "[a-zA-Z]+\\." str)
      (result:suffix 'LITERAL location
        (string-trim-both (match:substring current-match))))
    ; Literals -- words start with a '
    ((has-match? "'[a-zA-Z]+\\b" str)
      (result:suffix 'LITERAL location
        (substring (string-trim-both (match:substring current-match)) 1)))
    ((has-match? "[a-zA-Z-]+\\b" str)
      (if (is-lemma? (string-trim-both (match:substring current-match)))
        (result:suffix 'LEMMA location
          (string-trim-both (match:substring current-match)))
        ; Literals, words in the pattern that are not in their canonical forms
        (result:suffix 'LITERAL location
          (string-trim-both (match:substring current-match)))))
    ((has-match? "[0-9]+[0-9.]*\\b" str)
      (result:suffix 'NUM location
        (string-trim-both (match:substring current-match))))
    ((has-match? "[|]" str)
      (result:suffix 'VLINE location
        (string-trim-both (match:substring current-match))))
    ((has-match? "," str)
      (result:suffix 'COMMA location
        (string-trim-both (match:substring current-match))))
    ; This should always be near the end, because it is broadest of all.
    ((has-match? "[~’'._!?0-9a-zA-Z-]+" str)
      (result:suffix 'STRING location
        (string-trim-both (match:substring current-match))))
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
    (CONCEPT REACTIVE-RULE REJOINDER PROACTIVE-RULE URGE ORD-GOAL GOAL RGOAL COMMENT
     SAMPLE_INPUT PARALLEL-RULES LINK-CONCEPT RLINK-CONCEPT GLOBAL-DEFAULT-RULE
      (right: LPAREN LSBRACKET << ID VAR * ^ < LEMMA LITERAL LITERAL_APOS NUM DICTKEY
              STRING *~n *n UVAR MVAR MOVAR EQUAL NOT RESTART LBRACE VLINE COMMA
              SET_DELAY)
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
      (global-default-rule) : $1
      (urge) : (set-initial-urge $1)
      (goal) : (create-top-lv-goal $1)
      (ordered-goal) : (create-top-lv-goal $1 #t)
      (rule-goal) : (create-rule-lv-goal $1)
      (link-concept) : (create-top-lv-link-concepts $1)
      (rule-link-concept) : (create-rule-lv-link-concepts $1)
      (PARALLEL-RULES) : (create-top-lv-goal (list (cons "Parallel-Rules" 1)))
      (reactive-rule) : $1
      (proactive-rule) : $1
      (rejoinder) : $1
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
      (CONCEPT ID declaration-sequence) : (create-concept $2 $3)
      (UVAR EQUAL strs) : (create-user-variable $1 $3)
    )

    (declaration-sequence
      (LPAREN declaration-members RPAREN) :
        (eval-string (format #f "(list ~a)" $2))
      (LSBRACKET declaration-members RSBRACKET) :
        (eval-string (format #f "(list ~a)" $2))
    )

    (declaration-members
      (declaration-member) : $1
      (declaration-members declaration-member) : (format #f "~a ~a" $1 $2)
    )

    (declaration-member
      (lemma) : $1
      (literal) : $1
      (literal-apos) : $1
      (phrase) : $1
      (concept) : $1
      (sequence) : $1
      (DICTKEY) :
        (format #f "(cons 'dictkey (cons \"~a\" \"~a\"))" (car $1) (cdr $1))
    )

    (global-default-rule
      (GLOBAL-DEFAULT-RULE GDR-TYPE context action) : (create-default-rule $2 $3 $4)
      (GLOBAL-DEFAULT-RULE GDR-TYPE context) : (create-default-rule $2 $3 (list))
      (GLOBAL-DEFAULT-RULE GDR-TYPE action) : (create-default-rule $2 (list) $3)
    )

    ; For global default rule
    (GDR-TYPE
      (REACTIVE-RULE) : $1
      (PROACTIVE-RULE) : $1
    )

    (reactive-rule
      (REACTIVE-RULE str context action) : (create-rule $3 $4 $2 $1)
      (REACTIVE-RULE context action) : (create-rule $2 $3 "" $1)
    )

    (proactive-rule
      ; Note, do not support a proactive rule that has a
      ; label but no context -- it's ambiguous to determine
      ; whether it's really a label or just the first
      ; word of the "action"
      (PROACTIVE-RULE str context action) : (create-rule $3 $4 $2 $1)
      (PROACTIVE-RULE context action) : (create-rule $2 $3 "" $1)
      (PROACTIVE-RULE action) : (create-rule (list) $2 "" $1)
    )

    (rejoinder
      (REJOINDER str context action) : (create-rule $3 $4 $2 $1)
      (REJOINDER context action) : (create-rule $2 $3 "" $1)
    )

    (urge
      (URGE LPAREN goal-members RPAREN) :
        (eval-string (format #f "(list ~a)" $3))
    )

    (ordered-goal
      (ORD-GOAL LPAREN goal-members RPAREN) :
        (eval-string (format #f "(list ~a)" $3))
    )

    (rule-goal
      (RGOAL LPAREN goal-members RPAREN) :
        (eval-string (format #f "(list ~a)" $3))
    )

    (goal
      (GOAL LPAREN goal-members RPAREN) :
        (eval-string (format #f "(list ~a)" $3))
    )

    (goal-members
      (goal-member) : $1
      ; Support both space (backward compatibility) and comma as the delimiter
      ; For example:
      ; goal: (novelty=0.67 please_user=0.4)
      ; goal: (novelty=0.67, please_user=0.4)
      (goal-members goal-member) : (format #f "~a ~a" $1 $2)
      (goal-members COMMA goal-member) : (format #f "~a ~a" $1 $3)
    )

    (goal-member
      (LITERAL EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
      (LITERAL_APOS EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
      (LEMMA EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
      (STRING EQUAL NUM) : (format #f "(cons \"~a\" ~a)" $1 $3)
    )

    (link-concept
      (LINK-CONCEPT LPAREN link-concept-members RPAREN) : (string-split $3 #\,)
    )

    (rule-link-concept
      (RLINK-CONCEPT LPAREN link-concept-members RPAREN) : (string-split $3 #\,)
    )

    ; Can only use comma as the delimiter for link-concept
    (link-concept-members
      (strs) : $1
      (link-concept-members COMMA strs) : (format #f "~a,~a" $1 $3)
    )

    (context
      (context-pattern) : (eval-string (format #f "(list ~a)" $1))
    )

    (context-pattern
      (LPAREN RPAREN) : "(cons 'empty-context (list))"
      (LPAREN negation RPAREN) : $2
      (LPAREN context-terms RPAREN) : $2
      (LPAREN negation context-terms RPAREN) : (format #f "~a ~a" $2 $3)
    )

    (context-terms
      (context-term) : $1
      (context-terms context-term) : (format #f "~a\n~a" $1 $2)
      (context-terms enter) : $1
      (NEWLINE context-terms) : $2
      (context-terms COMMENT) : $1
      (COMMENT context-terms) : $2
    )

    (context-term
      (<) : "(cons 'anchor-start \"<\")"
      (>) : "(cons 'anchor-end \">\")"
      (wildcard) : $1
      (lemma) : $1
      (literal) : $1
      (literal-apos) : $1
      (phrase) : $1
      (concept) : $1
      (variable) : $1
      (user-variable) : $1
      (function) : $1
      (function-compare) : $1
      (choice) : $1
      (optional) : $1
      (variable ? concept) :
        (format #f "(cons 'is_member (list ~a ~a))" $1 $3)
      (sequence) : $1
      (unordered-matching) : $1
    )

    (action
      (action-terms) :
        (eval-string (format #f "(list (cons 'action (list ~a)))" $1))
    )

    (action-terms
      (action-term) : $1
      (action-terms action-term) : (format #f "~a ~a" $1 $2)
      (action-terms enter) : $1
      (NEWLINE action-terms) : $2
      (action-terms COMMENT) : $1
      (COMMENT action-terms) : $2
    )

    (action-term
      (?) : "(cons 'str \"?\")"
      (NOT) : "(cons 'str \"!\")"
      (COMMA) : "(cons 'str \",\")"
      (DQUOTE) : "(cons 'str \"\\\"\")"
      (LEMMA) : (format #f "(cons 'str \"~a\")" $1)
      (LITERAL) : (format #f "(cons 'str \"~a\")" $1)
      (LITERAL_APOS) : (format #f "(cons 'str \"~a\")" $1)
      (NUM) : (format #f "(cons 'str \"~a\")" $1)
      (STRING) : (format #f "(cons 'str \"~a\")" $1)
      (variable) : $1
      ; e.g. $username
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
      ; e.g. $username=Bob
      (UVAR EQUAL str) :
        (format #f "(cons 'assign_uvar (list \"~a\" (cons 'str \"~a\")))" $1 $3)
      ; e.g. $username='_0
      (UVAR EQUAL variable-grounding) :
        (format #f "(cons 'assign_uvar (list \"~a\" ~a))" $1 $3)
      ; e.g. $username=^get_name()
      (UVAR EQUAL function) :
        (format #f "(cons 'assign_uvar (list \"~a\" ~a))" $1 $3)
      (function) : $1
      (tts-feature) : $1
      (SET_DELAY) : (format #f "(cons 'set-delay \"~a\")" $1)
      (LSBRACKET action-terms RSBRACKET) :
        (format #f "(cons 'action-choices (list ~a))" $2)
    )

    (lemma
      (LEMMA) : (format #f "(cons 'lemma \"~a\")" $1)
    )

    (literal
      (LITERAL) : (format #f "(cons 'word \"~a\")" $1)
      (NUM) : (format #f "(cons 'word \"~a\")" $1)
      (STRING) : (format #f "(cons 'word \"~a\")" $1)
    )

    (literal-apos
      (LITERAL_APOS) : (format #f "(cons 'word-apos \"~a\")" $1)
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
      (LITERAL_APOS) : $1
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
      (LSBRACKET choice-pred-terms RSBRACKET) :
        (format #f "(cons 'seqor (list ~a))" $2)
    )

    (choice-terms
      (choice-term) : $1
      (choice-terms choice-term) : (format #f "~a ~a" $1 $2)
    )

    (choice-term
      (lemma) : $1
      (literal) : $1
      (literal-apos) : $1
      (phrase) : $1
      (concept) : $1
      (sequence) : $1
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
    )

    (choice-pred-terms
      (choice-pred-term) : $1
      (choice-pred-terms choice-pred-term) : (format #f "~a ~a" $1 $2)
    )

    ; For example,
    ; u: ( [ ^is_word_perceived(stop) ^is_word_preceived(wait) ] )
    ; u: ( [ ^is_after_min(.2) * ] )
    (choice-pred-term
      (*) : "(cons 'anyinput (list))"
      (function) : $1
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
      (MVAR operator right-compare) :
        (format #f "(cons 'compare (list \"~a\" ~a ~a))"
          $2 (format #f "(cons 'get_lvar ~a)" $1) $3)
      (MOVAR operator right-compare) :
        (format #f "(cons 'compare (list \"~a\" ~a ~a))"
          $2 (format #f "(cons 'get_wvar ~a)" $1) $3)
    )

    (user-variable
      (UVAR) : (format #f "(cons 'uvar_eval \"~a\")" $1)
      (UVAR operator right-compare) :
        (format #f "(cons 'compare (list \"~a\" ~a ~a))"
          $2 (format #f "(cons 'get_uvar \"~a\")" $1) $3)
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
      (literal-apos) : $1
      (phrase) : $1
      (concept) : $1
    )

    (function-compare
      (function operator right-compare) :
        (format #f "(cons 'compare (list \"~a\" ~a ~a))" $2 $1 $3)
    )

    (function
      (^ str LPAREN args RPAREN) :
        (format #f "(cons 'function (list \"~a\" ~a))" $2 $4)
      (^ str LPAREN RPAREN) :
        (format #f "(cons 'function (list \"~a\"))" $2)
      (^ str) :
        (format #f "(cons 'function (list \"~a\"))" $2)
    )

    (strs
      (str) : $1
      (strs str) : (format #f "~a ~a" $1 $2)
    )

    (str
      (LEMMA) : $1
      (LITERAL) : $1
      (LITERAL_APOS) : $1
      (STRING) : $1
      (NUM) : $1
    )

    (args
      (arg) : $1
      (arg COMMA args) : (format #f "~a ~a" $1 $3)
    )

    (arg
      (alphanum-args) : $1
      (var-arg) : $1
    )

    ; For multi-worded arguments
    ; e.g. ^stimulate_concept(being alive)
    (alphanum-args
      (alphanum-arg) : $1
      (alphanum-args alphanum-arg) :
        (format #f "(cons 'arg \"~a\")"
          (string-append (cdr (eval-string $1)) " " (cdr (eval-string $2))))
    )

    (alphanum-arg
      (LEMMA) : (format #f "(cons 'arg \"~a\")" $1)
      (LITERAL) : (format #f "(cons 'arg \"~a\")" $1)
      (LITERAL_APOS) : (format #f "(cons 'arg \"~a\")" $1)
      (LITERAL NUM) : (format #f "(cons 'arg \"~a~a\")" $1 $2)
      (NUM) : (format #f "(cons 'arg \"~a\")" $1)
      (STRING) : (format #f "(cons 'arg \"~a\")" $1)
    )

    (var-arg
      (variable-grounding) : $1
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
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
      (literal-apos) : $1
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
      (literal-apos) : $1
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
      (str) : (format #f "(cons 'str \"~a\")" $1)
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
    )

    (right-compare
      (UVAR) : (format #f "(cons 'get_uvar \"~a\")" $1)
      (str) : (format #f "(cons 'str \"~a\")" $1)
      (DQUOTE phrase-terms DQUOTE) : (format #f "(cons 'str \"~a\")" $2)
      (concept) : $1
      (function) : $1
    )

    (operator
      (<) : "smaller"
      (>) : "greater"
      (EQUAL EQUAL) : "equal"
      (< EQUAL) : "smaller_equal"
      (> EQUAL) : "greater_equal"
      (NOT EQUAL) : "not_equal"
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

(define-public (cs-parse line)
"
  Parse a text string in a Guile shell.
"
  (define parser (cs-parser))
  (parser (cs-lexer (open-input-string line)) error)
)

(define-public (cs-parse-file file)
"
  Parse a GHOST rule file in a Guile shell.
"
  (define parser (cs-parser))
  (define fp (open-file-input-port file))
  (set-port-encoding! fp "UTF-8")
  (parser (cs-lexer fp) error)
)

;; To convert things parsed by the parser into actual atomese.


; ----------
(define (order-terms TERMS)
"
  Order the terms in the intended order, and insert wildcards into
  appropriate positions of the sequence.
  No operation is needed if the pattern is supposed to be matched in
  any order.
  Finally make sure there is no meaningless consecutive
  wildcard/variable in the term list we are going to return, as that
  may cause a glob to be grounded in an unintentional way.
"
  (define (merge-wildcard INT1 INT2)
    (cons (max (car INT1) (car INT2))
          (cond ((or (negative? (cdr INT1)) (negative? (cdr INT2))) -1)
                (else (max (cdr INT1) (cdr INT2))))))

  (let* ((as (cons 'anchor-start "<"))
         (ae (cons 'anchor-end ">"))
         (wc (cons 'wildcard (cons 0 -1)))
         (unordered? (any (lambda (t) (equal? 'unordered-matching (car t))) TERMS))
         ; It's possible that the sequence does not having any word or concept etc,
         ; e.g. a rule may just be something like not-having-either-one-of-these-words
         (negation-only? (every (lambda (t) (equal? 'negation (car t))) TERMS))
         (empty-seq? (and (= 1 (length TERMS)) (equal? 'empty-context (caar TERMS))))
         (func-only? (and (> (length TERMS) 0)
           (every (lambda (t) (equal? 'function (car t))) TERMS)))
         (start-anchor? (any (lambda (t) (equal? as t)) TERMS))
         (end-anchor? (any (lambda (t) (equal? ae t)) TERMS))
         (start (if start-anchor? (cdr (member as TERMS)) (list wc)))
         (end (if end-anchor?
                  (take-while (lambda (t) (not (equal? ae t))) TERMS)
                  (list wc)))
         ; Start inserting the wildcards, including the implicit ones
         (term-lst
           (cond
             (unordered? TERMS)  ; Nothing needs to be done
             (func-only? TERMS)  ; Nothing needs to be done
             ((and start-anchor? end-anchor?)
              (if (equal? start-anchor? end-anchor?)
                  ; If they are equal, we are not expecting
                  ; anything else, either one of them is
                  ; the whole sequence
                  (drop-right start 1)
                  ; If they are not equal, put a wildcard
                  ; in between them
                  (append start (list wc) end)))
             ; If there is only a start-anchor, append it and
             ; a wildcard with the main-seq
             (start-anchor?
               (let ((before-anchor-start
                       (take-while (lambda (t) (not (equal? as t))) TERMS)))
                    (if (null? before-anchor-start)
                        (append start end)
                        ; In case there are terms before anchor-start,
                        ; get it and add an extra wildcard
                        (append start (list wc) before-anchor-start end))))
             ; If there is only an end-anchor, the main-seq should start
             ; with a wildcard, follow by another wildcard and finally
             ; the end-seq
             (end-anchor?
               (let ((after-anchor-end (cdr (member ae TERMS))))
                 (if (null? after-anchor-end)
                     (append start end)
                     ; In case there are still terms after anchor-end,
                     ; get it and add an extra wildcard
                     (append start after-anchor-end (list wc) end))))
             ; Just having one wildcard is enough for an empty sequence
             (negation-only? (append TERMS (list wc)))
             ; If the context is empty, not even a wildcard is needed
             (empty-seq? TERMS)
             ; If there is no anchor, the main-seq should start and
             ; end with a wildcard
             (else (append (list wc) TERMS (list wc))))))
        ; Remove, if any, wildcards that are not needed
        (fold-right
          (lambda (term prev)
            (cond ; If there are two consecutive wildcards, e.g.
                  ; (wildcard 0 . -1) (wildcard 3. 5)
                  ; merge them
                  ((and (equal? 'wildcard (car term))
                        (equal? 'wildcard (car (first prev))))
                   (cons (cons 'wildcard
                               (merge-wildcard (cdr term) (cdr (first prev))))
                         (cdr prev)))
                  ; If there is a variable that matches to "zero or more"
                  ; followed by a wildcard, e.g.
                  ; (variable (wildcard 0 . -1)) (wildcard 3 . 6)
                  ; return the variable
                  ((and (equal? 'variable (car term))
                        (equal? (cadr term) (cons 'wildcard (cons 0 -1)))
                        (equal? 'wildcard (car (first prev))))
                   (cons term (cdr prev)))
                  ; Same as the above, but for optionals
                  ((and (equal? 'optionals (car term))
                        (equal? 'wildcard (car (first prev))))
                   (cons term (cdr prev)))
                  ; Similarly if there is a wildcard followed by a variable
                  ; that matches to "zero or more", e.g.
                  ; (wildcard 0 . -1) (variable (wildcard 0 . -1))
                  ; return the variable, i.e. ignore this wildcard
                  ((and (equal? 'wildcard (car term))
                        (equal? 'variable (car (first prev)))
                        (equal? (cadr (first prev)) (cons 'wildcard (cons 0 -1))))
                   prev)
                  ; Same as the above, but for optionals
                  ((and (equal? 'wildcard (car term))
                        (equal? 'optionals (car (first prev))))
                   prev)
                  ; Otherwise accept and append it to the list
                  (else (cons term prev))))
          (list (last term-lst))
          (list-head term-lst (- (length term-lst) 1)))))

; ----------
(define (process-pattern-terms TERMS)
"
  Generate the atomese (i.e. the variable declaration and the pattern)
  for each of the TERMS in the pattern.
"
  (define is-unordered? #f)
  (define has-words? #f)
  (define specificity 0)

  ; A word has at least the following conditions to be satisfied
  ; - one and only one word -> i.e. lower & upper bound
  ; - in a particular form of a particular lemma
  (define spec-word 4)

  ; A lemma has at least the following conditions to be satisfied
  ; - one and only one word -> i.e. lower & upper bound
  ; - a particular lemma (in any form)
  (define spec-lemma 3)

  ; A concept has at least the following conditions to be satisfied
  ; - be grounded to one or more words -> i.e. lower bound
  ; - be related to a particular "concept"
  (define spec-concept 2)

  (define (process terms)
    (define v '())
    (define c '())
    (define ws '())
    (define ls '())
    (define (update-lists t)
      (set! v (append v (list-ref t 0)))
      (set! c (append c (list-ref t 1)))
      (set! ws (append ws (list-ref t 2)))
      (set! ls (append ls (list-ref t 3))))

    (for-each (lambda (t)
      (cond ((equal? 'unordered-matching (car t))
             (update-lists (process (cdr t)))
             ; TODO: The specificity of ordered vs unordered should be
             ; considered as well
             (set! is-unordered? #t))
            ((equal? 'word (car t))
             (update-lists (word (cdr t)))
             (set! specificity (+ specificity spec-word))
             (set! has-words? #t))
            ((equal? 'word-apos (car t))
             (update-lists (word-apos (cdr t)))
             ; Its specificity should be the same as a word
             (set! specificity (+ specificity spec-word))
             (set! has-words? #t))
            ((equal? 'lemma (car t))
             (update-lists (lemma (cdr t)))
             (set! specificity (+ specificity spec-lemma))
             (set! has-words? #t))
            ((equal? 'phrase (car t))
             (update-lists (phrase (cdr t)))
             ; The specificity of a phrase depends on the number of
             ; words it has and each of those words is considered
             ; as a "word-term"
             (set! specificity (+ specificity
               (* (length (string-split (cdr t) #\sp)) spec-word)))
             (set! has-words? #t))
            ((equal? 'concept (car t))
             (update-lists (concept (cdr t)))
             (set! specificity (+ specificity spec-concept))
             (set! has-words? #t))
            ((equal? 'choices (car t))
             (update-lists (choices (cdr t)))
             ; The specificity of choices should be the term in
             ; the list (of choices) with the least specificity
             ; Potentially "concept" could have done this way too,
             ; though concept may change dynamically at runtime
             ; but this will not
             (set! specificity (+ specificity
               (fold
                 (lambda (tc min-spec)
                   (cond
                     ; Only included the terms that are less specific
                     ; than a phrase
                     ((equal? 'concept (car tc)) spec-concept)
                     ((and (equal? 'lemma (car tc))
                           (< spec-lemma min-spec))
                      spec-lemma)
                     ((and (equal? 'word (car tc))
                           (< spec-word min-spec))
                      spec-word)
                     (else min-spec)))
                 ; The most specific term possibility exist in choices
                 ; is a phrase, so check and see if there is any phrase
                 ; and get its specificity, or just take the specificity
                 ; of a word if not
                 (fold
                   (lambda (ph max-spec)
                     (cond
                       ((and (equal? 'phrase (car ph))
                             (> (* (length (string-split (cdr ph) #\sp)) spec-word)
                                max-spec))
                        (* (length (string-split (cdr ph) #\sp)) spec-word))
                       (else max-spec)))
                   spec-word
                   (cdr t)
                 )
                 (cdr t))))
             (set! has-words? #t))
            ((equal? 'optionals (car t))
             (update-lists (optionals (cdr t)))
             ; The specificity of optionals is almost identical to concept,
             ; except that it's not requied to be presence (viz optional)
             (set! specificity (+ specificity (- spec-concept 1)))
             (set! has-words? #t))
            ((equal? 'negation (car t))
             (update-lists (negation (cdr t)))
             ; Negation has a condition of not having a particular
             ; term or a list of terms to be presented in the pattern
             (set! specificity (+ specificity 1))
             (set! has-words? #t))
            ((equal? 'wildcard (car t))
             (update-lists (wildcard (cadr t) (cddr t)))
             ; The specificity of a wildcard depands on the interval
             ; For a pure wildcard, specificity = 0
             ; For a range-restricted wildcard (zero to n),
             ; upper bound is given so specificity = 1
             ; For a precise wildcard (n and only n), both lower and
             ; upper bound is given so specificity = 2
             (if (> (cadr t) 0) (set! specificity (1+ specificity)))
             (if (> (cddr t) -1) (set! specificity (1+ specificity)))
             (set! has-words? #t))
            ((equal? 'variable (car t))
             (update-lists (process (cdr t)))
             (set! pat-vars (append pat-vars (last-pair ws))))
            ((equal? 'uvar_exist (car t))
             (set! c (append c (list (uvar-exist? (cdr t)))))
             ; User variable has a condition of checking if a particular
             ; user variable has been defined
             (set! specificity (+ specificity 1))
             (set! has-words? #t))
            ((equal? 'uvar_equal (car t))
             (set! c (append c (list (uvar-equal? (cadr t) (caddr t)))))
             ; uvar_equal should have a specificity of a word as they
             ; have the same set of conditions to be satisfied
             (set! specificity (+ specificity spec-word))
             (set! has-words? #t))
            ((equal? 'function (car t))
             ; The specificity of a function / predicate depends really
             ; on what that function is doing (and it could be anything)
             ; For now, just assume the specificity of a function is 1.
             (set! specificity (+ specificity 1))
             (set! c (append c
               (list (context-function (cadr t)
                 (map (lambda (a)
                   (cond ((equal? 'get_wvar (car a))
                          (list-ref pat-vars (cdr a)))
                         ((equal? 'get_lvar (car a))
                          (get-var-lemmas (list-ref pat-vars (cdr a))))
                         ((equal? 'get_uvar (car a))
                          (get-user-variable (cdr a)))
                         (else (ConceptNode (cdr a)))))
                   (cddr t)))))))
            ((equal? 'sequence (car t))
             (let ((pt (process (cdr t))))
                  ; Wrap the sequences with a ListLink
                  (list-set! pt 2 (list (List (list-ref pt 2))))
                  (list-set! pt 3 (list (List (list-ref pt 3))))
                  (update-lists pt)))
            ; If it's an empty context, e.g. "u: ()"
            ; it's always true and should be triggered
            ; even if there is no perception input
            ((equal? 'empty-context (car t))
              (set! c (list (TrueLink))))
            (else (begin
              (cog-logger-warn ghost-logger
                "Feature not supported: \"(~a ~a)\"" (car t) (cdr t))
              (throw 'FeatureNotSupported (car t) (cdr t))))))
      terms)
    (list v c ws ls))

  (define (generate-eval pred seq)
    (Evaluation pred (List (Variable "$S") (List (flatten-list seq)))))

  ; Start the processing
  (define terms (process TERMS))
  (define vars (list-ref terms 0))
  (define conds (list-ref terms 1))
  (define word-seq (list-ref terms 2))
  (define lemma-seq (list-ref terms 3))

  ; If there is no word in the pattern, then don't put the seqs
  ; and the other sentence related stuffs in the context
  (if has-words?
    (begin
      (set! vars (append vars (list
        (TypedVariable (Variable "$S") (Type "SentenceNode"))
        (TypedVariable (Variable "$P") (Type "ParseNode")))))
      (set! conds (append conds (list
        (Parse (Variable "$P") (Variable "$S"))
        (State ghost-curr-proc (Variable "$S")))))
      (if is-unordered?
        ; Generate an EvaluationLink for each of the term in the seq
        ; if it's an unordered match
        (begin
          (for-each
            (lambda (t)
              (let ((wc (wildcard 0 -1)))
                (set! vars (append vars (list-ref wc 0)))
                (set! conds (append conds (list (generate-eval ghost-word-seq
                  (list (car (list-ref wc 2)) t (car (list-ref wc 3)))))))))
            word-seq)
          (if (not ghost-with-ecan)
            (for-each
              (lambda (t)
                (let ((wc (wildcard 0 -1)))
                  (set! vars (append vars (list-ref wc 0)))
                  (set! conds (append conds (list (generate-eval ghost-lemma-seq
                    (list (car (list-ref wc 2)) t (car (list-ref wc 3)))))))))
              lemma-seq)))
        ; Otherwise it's an ordered match
        (begin
          (set! conds (append conds (list
            (generate-eval ghost-word-seq word-seq))))
          (if (not ghost-with-ecan)
            (set! conds (append conds (list
              (generate-eval ghost-lemma-seq lemma-seq))))))))
    ; See below
    (if (not ghost-with-ecan)
      (MemberLink (car conds) ghost-no-constant)))

  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  (if (and (not ghost-with-ecan)
        (equal? (length lemma-seq)
          (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                          lemma-seq))))
      (MemberLink (List lemma-seq) ghost-no-constant))

  (list vars conds has-words? specificity)
)

; ----------
(define (process-action ACTION RULENAME)
"
  Generate the atomese for each of the terms in ACTION.
  RULENAME is the alias assigned to the rule.
"
  ; The system functions that are commonly used
  (define reuse #f)
  (define keep (topic-has-feature? rule-topic "keep"))

  ; The GroundedSchemaNode that will be used
  (define gsn-action (GroundedSchema "scm: ghost-execute-action"))

  (define (to-atomese actions)
    (define choices '())
    (append
      ; Iterate through the output word-by-word
      (map (lambda (n)
        (cond ; Gather all the action choices, i.e. a list of actions
              ; available but only one of them will be executed
              ((equal? 'action-choices (car n))
               (set! choices (append choices (list (List (to-atomese (cdr n))))))
               '())
              ; Generate the DefinedSchema once we have finished going through
              ; all the choices
              ((not (null? choices))
               (let ((ac (action-choices choices)))
                    (set! choices '())
                    ; Generate the atoms for the current one
                    (append (list ac) (to-atomese (list n)))))
              ; The grounding of a variable in original words, e.g. '_0
              ((equal? 'get_wvar (car n))
               (list-ref pat-vars (cdr n)))
              ; The grounding of a variable in lemmas, e.g. _0
              ((equal? 'get_lvar (car n))
               (get-var-lemmas (list-ref pat-vars (cdr n))))
              ; Get the value of a user variable, e.g. $name
              ((equal? 'get_uvar (car n)) (get-user-variable (cdr n)))
              ; Assign a value to a user variable, e.g. $name=Bob
              ((equal? 'assign_uvar (car n))
               (set-user-variable (cadr n) (car (to-atomese (cddr n)))))
              ; A system function -- keep
              ((and (equal? 'function (car n))
                    (equal? "keep" (cadr n)))
               (set! keep #t)
               (list))
              ; A system function -- reuse
              ; First of all, try to see if the rule has been created in
              ; the AtomSpace, and get its action directly if so
              ; Otherwise, check the rule-alist to see if the rule is
              ; defined in the same file being parsed
              ((and (equal? 'function (car n))
                    (equal? "reuse" (cadr n)))
               (let* ((label (cdaddr n))
                      (reused-rule (get-rule-from-label label)))
                 (if (null? reused-rule)
                   (let ((reused-rule-from-alist (assoc-ref rule-alist label)))
                     (if (null? reused-rule-from-alist)
                       (cog-logger-error ghost-logger
                         "Please make sure the rule with label \"~a\" is defined!" label)
                       (begin
                         (cog-logger-debug ghost-logger
                           "Found the rule \"~a\" in the rule-alist" label)
                         (set! reuse #t)
                         (process-action
                           ; The 2nd item in the list is the action
                           (list-ref reused-rule-from-alist 1) label))))
                   (begin
                     (set! reuse #t)
                     (psi-get-action reused-rule)))))
              ; Other functions
              ((equal? 'function (car n))
               (action-function (cadr n) (to-atomese (cddr n))))
              ; TTS feature
              ((equal? 'tts-feature (car n))
               (tts-feature (to-atomese (cdr n))))
              ; Set STT cutoff time
              ((equal? 'set-delay (car n))
               (set-delay (Number
                 (match:substring (string-match "[0-9]+" (cdr n))))))
              (else (Word (cdr n)))))
        actions)
      (if (null? choices)
          '()
          (list (action-choices choices)))))

  ; A typical action of a GHOST rule looks like this:
  ;
  ; (TrueLink
  ;   (ExecutionOutputLink
  ;     (GroundedSchemaNode "scm: ghost-execute-action")
  ;     (ListLink
  ;        ... words & functions ...
  ;     )
  ;   )
  ;   ... and maybe more ...
  ; and here we try to get the words & functions
  ; from the rule being reused and append it to
  ; rule that uses the reuse function, as well as
  ; the additional ones that are in the same TrueLink
  ; TODO: Handle variables as well
  (define (get-reused-action atomese)
    (define action-atomese
      (append-map
        (lambda (x)
          (cond ; "x" could just be a list if ^keep() is used
                ; in the same rule, skip it that's the case
                ((and keep (list? x) (null? x)) (list))
                ((equal? 'TrueLink (cog-type x))
                 (get-reused-action (cog-outgoing-set x)))
                ((and (equal? 'ExecutionOutputLink (cog-type x))
                      (equal? gsn-action (gar x)))
                 (get-reused-action (cog-outgoing-set (gdr x))))
                (else (list x))))
        atomese))
      ; Filter out duplicate PutLinks in the action -- the ones that
      ; are updating the same state, e.g. if there are (Put (State A) B)
      ; and (Put (State A) C) in a list, then only (Put (State A) C)
      ; is kept as it's the last one in the list
      ; Filtering may be needed when "reuse" is called more than one time,
      ; either in a single rule, or other rule in the chain
      ; The fold-right and reverse are there to perserve the execution order
      (reverse
        (fold-right
          (lambda (atom rtn)
            (if (and (equal? 'PutLink (cog-type atom))
                     (equal? 'StateLink (cog-type (gar atom)))
                     (find (lambda (x) (equal? (gar atom) (gar x))) rtn))
              rtn
              (append rtn (list atom))))
          (list)
          action-atomese)))

  (define action-atomese (to-atomese (cdar ACTION)))

  (True
    (list
      (ExecutionOutput
        gsn-action
        (List
          (if reuse
            (get-reused-action action-atomese)
            action-atomese)))
      ; Keep a record of which rule is the last executed one, just for rejoinders
      ; And when a "reuse" is used, it becomes slightly more complicated
      ; The expected behavior is that, when (the action of) a rule is reused,
      ; the rule will be considered as fired, so mark the last executed rule
      ; as the reused one instead of the one that calls the reuse function,
      ; so that the rejoinders (if any) of the reused rule can be triggered
      ; correctly
      (if reuse
        (list)
        (Put (State ghost-last-executed (Variable "$x"))
             (Concept RULENAME)))
      ; The default behavior is to not executed the same action
      ; more than once -- update the TV strength to zero so that
      ; the action selector won't select it again
      ; Except if the rule is supposed to be kept or it's
      ; a rejoinder (rejoinders are kept by default, because it
      ; won't be triggered anyway if the parent is not triggered
      ; and if the parent is triggered, one would expect the
      ; rejoinders can be triggered too for the next input)
      (if (or keep (equal? (assoc-ref rule-type-alist RULENAME) strval-rejoinder))
          (list)
          (ExecutionOutput
            (GroundedSchema "scm: ghost-update-rule-strength")
            ; Note: This is generated by psi-rule-set-alias!
            (List (Concept RULENAME)
                  (Number 0))))
      ; Keep a record of which rules have been executed
      (ExecutionOutput
        (GroundedSchema "scm: ghost-record-executed-rule")
        (List (Concept RULENAME)))
      ; Set the current topic, for backward compatibility
      (if ghost-with-ecan
        (list)
        (Put (State ghost-curr-topic (Variable "$x"))
             rule-topic)))))

; ----------
(define (process-goal GOAL)
"
  Go through each of the goals, including the shared ones.
"
  (if (and (null? GOAL) (null? top-lv-goals))
      (begin
        (cog-logger-warn ghost-logger
          "Did you forget to link a goal to the rule?")
        (list (cons (ghost-prefix "Default Goal") .9)))
      ; The shared goals will be overwritten if the same goal is given
      ; to this rule
      (append GOAL
        (remove (lambda (sg) (any (lambda (g) (equal? (car sg) (car g))) GOAL))
                top-lv-goals))))

; ----------
(define (process-type TYPE NAME)
"
  Figure out what the type of the rule is, generate the needed atomese, and
  return the type as a StringValue.
"
  (cond ((or (equal? #\u TYPE)
             (equal? #\s TYPE))
         (set! rule-type-alist
           (assoc-set! rule-type-alist NAME strval-responder))
         (list '() '()))
        ((equal? #\? TYPE)
         (set! rule-type-alist
           (assoc-set! rule-type-alist NAME strval-responder))
         (let ((var (Variable (gen-var "Interpretation" #f))))
           (list
             (list (TypedVariable var (Type "InterpretationNode")))
             (list
               (InterpretationLink var (Variable "$P"))
               (ChoiceLink
                 (InheritanceLink var
                   (DefinedLinguisticConcept "InterrogativeSpeechAct"))
                 (InheritanceLink var
                   (DefinedLinguisticConcept "TruthQuerySpeechAct")))))))
        ((equal? #\r TYPE)
         (set! rule-type-alist
           (assoc-set! rule-type-alist NAME strval-random-gambit))
         (list '() '()))
        ((equal? #\t TYPE)
         (set! rule-type-alist
           (assoc-set! rule-type-alist NAME strval-gambit))
         (list '() '()))
        ((null? rule-hierarchy)
         ; If we are here, it has to be a rejoinder, so make sure
         ; rule-hierarchy is not empty, i.e. a responder should
         ; be defined in advance
         (throw (ghost-prefix
           "Please define a responder first before defining a rejoinder.")))
        ; For rejoinders, put the condition (the last rule executed is
        ; the parent of this rejoinder) in the pattern of the rule
        (else (let ((var (Variable (gen-var "GHOST-rule" #f)))
                    (lv (get-rejoinder-level TYPE)))
          (set! rule-type-alist
            (assoc-set! rule-type-alist NAME strval-rejoinder))
          (list
            (list (TypedVariable var (Type "ConceptNode")))
            (list (State ghost-last-executed var)
                  (Equal var
                    (Concept (last (list-ref rule-hierarchy (- lv 1)))))))))))

; ----------
; Key used to set the value of psi-rules as either (stv 1 1) or (stv 0 1).
(define handles-sent-key (Predicate "handles-sentence-input"))

; ----------
(define (handles-sent! rule)
"
  set-handles-sent! RULE

  Returns the RULE after setting the handles-sentence-input value to (stv 1 1).
"
  (cog-set-value! rule handles-sent-key (stv 1 1)))

; ----------
(define (handles-sent? rule)
"
  handles-sent? RULE

  Returns (stv 1 1) if the RULE is tagged to handle sentences, else
  it returns (stv 0 1).
"
  (let ((result (cog-value rule handles-sent-key)))
    (if (null? result)
      (stv 0 1)
      result
    )
  ))

; ----------
(define (process-rule-stack)
"
  Instantiate the rules accumulated in rule-label-list.
"
  (for-each
    (lambda (l)
      (apply instantiate-rule (assoc-ref rule-alist l)))
    rule-label-list)

  ; Clear the states
  (set! rule-label-list '())
  (set! rule-alist '())
  (set! rule-hierarchy '())
)

; ----------
(define (create-rule PATTERN ACTION GOAL NAME TYPE)
"
  Top level translation function.

  PATTERN, ACTION, and GOAL are the basic components of a psi-rule,
  correspond to context, procedure, and goal respectively.

  NAME is like a label of a rule, so that one can reference this rule
  by using it.

  TYPE is a grouping idea from ChatScript, e.g. responders, rejoinders,
  gambits etc.
"
  ; Label the rule with NAME, if given, generate one otherwise
  (define rule-name
    (if (string-null? NAME)
      (string-append "GHOST-rule-" (random-string 36))
      NAME))

  (set! rule-label-list (append rule-label-list (list rule-name)))

  (set! rule-alist
    (assq-set! rule-alist rule-name
      (list PATTERN ACTION (process-goal GOAL) GOAL rule-name TYPE is-rule-seq?)))
)

; ----------
(define (instantiate-rule PATTERN ACTION ALL-GOALS RULE-LV-GOALS NAME TYPE ORDERED?)
"
  To process and create the rule in the AtomSpace.
"
  (define (add-to-rule-hierarchy LV RULE)
    ; Reset the rule hierarchy if it's not a rejoinder
    (if (= LV 0)
      (set! rule-hierarchy (list (list RULE)))
      (if (= (length rule-hierarchy) LV)
        (set! rule-hierarchy (append rule-hierarchy (list (list RULE))))
        (list-set! rule-hierarchy LV
          (append (list-ref rule-hierarchy LV) (list RULE))))))

  (define (set-next-rule PRULE CRULE KEY)
    (define val (cog-value PRULE KEY))
    (cog-set-value! PRULE KEY
      (if (null? val)
        (LinkValue CRULE)
        (apply LinkValue (append (cog-value->list val) (list CRULE))))))

  ; First of all, make sure the topic is set
  ; so that it can be used when we are processing the action
  (if (null? rule-topic)
    (set! rule-topic (create-topic "Default Topic")))

  ; Reset the list of local variables
  (set! pat-vars '())

  (let* ((proc-type (process-type TYPE NAME))
         (ordered-terms (order-terms PATTERN))
         (proc-terms (process-pattern-terms ordered-terms))
         (vars (append (list-ref proc-terms 0)
                       (list-ref proc-type 0)))
         (conds (append (list-ref proc-terms 1)
                        (list-ref proc-type 1)))
         (specificity (list-ref proc-terms 3))
         (type (assoc-ref rule-type-alist NAME))
         (action (process-action ACTION NAME))
         (is-rejoinder? (equal? type strval-rejoinder))
         (rule-lv (if is-rejoinder? (get-rejoinder-level TYPE) 0)))

    (cog-logger-debug ghost-logger "Context: ~a" ordered-terms)
    (cog-logger-debug ghost-logger "Procedure: ~a" ACTION)
    (cog-logger-debug ghost-logger "Goal: ~a" ALL-GOALS)

    ; Update the count -- how many rules we've seen under this top level goal
    ; Do it only if the rules are ordered and it's not a rejoinder
    (if (and ORDERED? (not is-rejoinder?))
      (begin
        (set! goal-rule-cnt (+ goal-rule-cnt 1))
        ; Force the rules defined in a sequence to be triggered
        ; in an ordered fashion
        ; Note: psi-action-executed? is not used here, because
        ; when (the action of) a rule is "reused", it will be
        ; considered as "used". But "reuse" is just about executing
        ; the action of another rule, it doesn't evaluate the
        ; context of a rule, i.e. it doesn't go through the
        ; PsiImplicator, so psi-action-executed? will return
        ; false for the reused rule even if its action has been
        ; executed already, which is not the behavior we want here
        ; TODO: Remove the geometric series as it is no longer needed?
        (if (> (length rule-hierarchy) 0)
          (let ((var (Variable (gen-var "GHOST-executed-rule" #f))))
            (set! vars (append vars (list
              (TypedVariable var (Type "ConceptNode")))))
            (set! conds (append conds (list
              (Evaluation ghost-rule-executed (List var))
              (Equal var (Concept (caar rule-hierarchy))))))))
    ))

    (map
      (lambda (goal)
        ; Create the rule
        (define a-rule
          (psi-rule
            (list (Satisfaction (VariableList vars) (And conds)))
            action
            ; Make sure the goal has been created
            (psi-goal (car goal)
              ; Check if an initial urge has been assigned to it
              (let ((urge (assoc-ref initial-urges (car goal))))
                (if urge (- 1 urge) default-urge)))
            ; Check if the goal is defined at the rule level
            ; If the rule is ordered, the weight should change
            ; accordingly as well
            (if (or (member goal RULE-LV-GOALS) (not ORDERED?))
              (stv (cdr goal) .9)
              (stv (/ (cdr goal) (expt 2 (+ rule-lv goal-rule-cnt))) .9))
            ghost-component))

        ; If the rule can possibly be satisfied by input sentence
        ; tag it as such.
        (if (list-ref proc-terms 2)
          (handles-sent! a-rule)
          a-rule)

        ; Label the rule
        (psi-rule-set-alias! a-rule NAME)

        ; Set the type
        (cog-set-value! a-rule ghost-rule-type type)

        ; Set how specific the context of the rule is
        (cog-set-value! a-rule ghost-context-specificity (FloatValue specificity))

        ; Associate it with its topic
        (if (not ghost-with-ecan)
          (Inheritance a-rule rule-topic))

        ; Keep track of the rule hierarchy, and link rules that
        ; are defined in a sequence
        (if is-rejoinder?
          ; If it's a rejoinder, its parent rule should be the
          ; last rule one level up in rule-hierarchy
          ; 'process-type' will make sure there is a responder
          ; defined beforehand so rule-hierarchy is not empty
          (begin
            (set-next-rule
              (get-rule-from-label
                (last (list-ref rule-hierarchy
                  (1- (get-rejoinder-level TYPE)))))
              a-rule ghost-next-rejoinder)
            (add-to-rule-hierarchy
              (get-rejoinder-level TYPE) NAME)
            ; Record the sequence number of the rejoinder
            ; This is used during matching, basically rejoinders is treated
            ; as a sequence, and the one defined first will be matched first
            ; if it satisfies the context
            (cog-set-value! a-rule ghost-rej-seq-num (FloatValue
              (length (list-ref rule-hierarchy (get-rejoinder-level TYPE))))))
          (begin
            ; If it's not a rejoinder, its parent rules should
            ; be the rules at every level that are still in
            ; the rule-hierarchy
            (if (and ORDERED? (not (null? rule-hierarchy)))
              (for-each
                (lambda (lv)
                  (for-each
                    (lambda (r)
                      (set-next-rule
                        (get-rule-from-label r)
                          a-rule ghost-next-responder))
                    lv))
                rule-hierarchy))
            (add-to-rule-hierarchy 0 NAME)))

        ; Connect words, concepts and predicates from the context
        ; directly to the rule via a HebbianLink
        (for-each
          (lambda (node) (AsymmetricHebbianLink node a-rule (stv 1 1)))
          (filter
            (lambda (x)
              (or (equal? ghost-word-seq x)
                  (equal? 'WordNode (cog-type x))
                  (equal? 'ConceptNode (cog-type x))
                  (equal? 'GroundedPredicateNode (cog-type x))))
            (append-map cog-get-all-nodes conds)))

        ; (cog-logger-debug ghost-logger "rule-hierarchy: ~a" rule-hierarchy)

        ; Return
        a-rule)
      ALL-GOALS)))

; ----------
(define (create-concept NAME MEMBERS)
"
  Create named concepts with explicit membership lists.

  NAME is the name of the concept, and the rest is the list of words
  and/or concepts that will be considered as the members of the concept.
"
  (map (lambda (m) (Reference m (Concept NAME)))
       (terms-to-atomese MEMBERS)))

(define (set-initial-urge URGES)
"
  Record the initial urge of a goal, which will be used during goal creation.
"
  (for-each
    (lambda (u)
      (set! initial-urges (assoc-set! initial-urges (car u) (cdr u)))
      ; Also, create the goal in the AtomSpace
      (psi-goal (car u) (- 1 (cdr u))))
    URGES
  )
)

(define* (create-top-lv-goal GOALS #:optional (ORDERED #f))
"
  Create a top level goal that will be shared among the rules under it.
"
  ; Actually create the goals in the AtomSpace
  (for-each
    (lambda (goal)
      (psi-goal (car goal)
        ; Check if an initial urge has been assigned to it
        (let ((urge (assoc-ref initial-urges (car goal))))
          (if urge (- 1 urge) default-urge))))
    GOALS)

  (set! top-lv-goals GOALS)
  (set! is-rule-seq? ORDERED)

  ; Reset the count when we see a new top level goal
  (set! goal-rule-cnt 0))

(define* (create-topic TOPIC-NAME #:optional (FEATURES (list)) (KEYWORDS (list)))
"
  Create a GHOST topic named TOPIC-NAME.

  FEATURES are the topic level features that will be applied to every
  single rules under this topic.

  KEYWORDS are kind of like the concepts related to this topic.
  Currently it doesn't have any effect on the action selection, so may
  be removed in the future.
"
  ; A topic will be defined when loading a (topic) file
  (set! rule-topic (Concept (ghost-prefix TOPIC-NAME)))
  (Inheritance rule-topic ghost-topic)

  ; The set of features associate with the topic
  ; The features will be stored as "values"
  (cog-set-value! rule-topic ghost-topic-feature
    (apply LinkValue (map (lambda (f) (StringValue f)) FEATURES)))

  ; The set of keywords associate with the topic
  (for-each (lambda (kw) (Member kw rule-topic)) (terms-to-atomese KEYWORDS))

  rule-topic)

(define (create-user-variable UVAR VAL)
"
  Define a new user variable.
"
  (ghost-set-user-variable (ghost-uvar UVAR) (List (Word VAL)))
)

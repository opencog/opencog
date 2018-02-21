;; To convert things parsed by the parser into actual atomese.

; ----------
; Shared variables for all terms
(define atomese-variable-template
  (list (TypedVariable (Variable "$S") (Type "SentenceNode"))
        (TypedVariable (Variable "$P") (Type "ParseNode"))))

; Shared conditions for all terms
(define atomese-condition-template
  (list (Parse (Variable "$P") (Variable "$S"))
        (State ghost-curr-proc (Variable "$S"))))

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
         ; or just functions
         (empty-seq? (every (lambda (t)
           (or (equal? 'negation (car t))
               (equal? 'function (car t)))) TERMS))
         (start-anchor? (any (lambda (t) (equal? as t)) TERMS))
         (end-anchor? (any (lambda (t) (equal? ae t)) TERMS))
         (start (if start-anchor? (cdr (member as TERMS)) (list wc)))
         (end (if end-anchor?
                  (take-while (lambda (t) (not (equal? ae t))) TERMS)
                  (list wc)))
         ; Start inserting the wildcards, including the implicit ones
         (term-lst (cond (unordered? TERMS)  ; Nothing needs to be done
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
                   (empty-seq? (append TERMS (list wc)))
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
             (set! is-unordered? #t))
            ((equal? 'word (car t))
             (update-lists (word (cdr t))))
            ((equal? 'lemma (car t))
             (update-lists (lemma (cdr t))))
            ((equal? 'phrase (car t))
             (update-lists (phrase (cdr t))))
            ((equal? 'concept (car t))
             (update-lists (concept (cdr t))))
            ((equal? 'choices (car t))
             (update-lists (choices (cdr t))))
            ((equal? 'optionals (car t))
             (update-lists (optionals (cdr t))))
            ((equal? 'negation (car t))
             (update-lists (negation (cdr t))))
            ((equal? 'wildcard (car t))
             (update-lists (wildcard (cadr t) (cddr t))))
            ((equal? 'variable (car t))
             (update-lists (process (cdr t)))
             (set! pat-vars (append pat-vars (last-pair ws))))
            ((equal? 'uvar_exist (car t))
             (set! c (append c (list (uvar-exist? (cdr t))))))
            ((equal? 'uvar_equal (car t))
             (set! c (append c (list (uvar-equal? (cadr t) (caddr t))))))
            ((equal? 'function (car t))
             (set! c (append c
               (list (context-function (cadr t)
                 (map (lambda (a)
                   (cond ((equal? 'get_wvar (car a))
                          (list-ref pat-vars (cdr a)))
                         ((equal? 'get_lvar (car a))
                          (get-var-lemmas (list-ref pat-vars (cdr a))))
                         ((equal? 'get_uvar (car a))
                          (get-user-variable (cdr a)))
                         (else (WordNode (cdr a)))))
                   (cddr t)))))))
            ((equal? 'sequence (car t))
             (let ((pt (process (cdr t))))
                  ; Wrap the sequences with a ListLink
                  (list-set! pt 2 (list (List (list-ref pt 2))))
                  (list-set! pt 3 (list (List (list-ref pt 3))))
                  (update-lists pt)))
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
        (for-each
          (lambda (t)
            (let ((wc (wildcard 0 -1)))
              (set! vars (append vars (list-ref wc 0)))
              (set! conds (append conds (list (generate-eval ghost-lemma-seq
                (list (car (list-ref wc 2)) t (car (list-ref wc 3)))))))))
          lemma-seq))
      ; Otherwise it's an ordered match
      (set! conds (append conds (list
        (generate-eval ghost-word-seq word-seq)
        (generate-eval ghost-lemma-seq lemma-seq)))))

  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  ; TODO: Mark the ones with only functions in the context as well
  ; XXX TODO: This can impede the performance if we have many of these
  ;           in the rulebase
  (if (equal? (length lemma-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              lemma-seq)))
      (MemberLink (List lemma-seq) ghost-no-constant))

  (list vars conds))

; ----------
(define (process-action ACTION RULENAME)
"
  Generate the atomese for each of the terms in ACTION.
  RULENAME is the alias assigned to the rule.
"
  ; The system functions that are commonly used
  (define reuse #f)
  (define keep (topic-has-feature? rule-topic "keep"))

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
              ((and (equal? 'function (car n))
                    (equal? "reuse" (cadr n)))
               (set! reuse #t)
               (action-function (cadr n) (to-atomese (cddr n))))
              ; Other functions
              ((equal? 'function (car n))
               (action-function (cadr n) (to-atomese (cddr n))))
              ; TTS feature
              ((equal? 'tts-feature (car n))
               (tts-feature (to-atomese (cdr n))))
              (else (Word (cdr n)))))
        actions)
      (if (null? choices)
          '()
          (list (action-choices choices)))))
  (define action-atomese (to-atomese (cdar ACTION)))
  (True (if reuse
            action-atomese
            (list (ExecutionOutput
                    (GroundedSchema "scm: ghost-execute-action")
                    (List action-atomese))
                  (if (not keep)
                      ; The default behavior is to not executed the
                      ; same action more than once -- update the
                      ; TV strength to zero so that the action
                      ; selector won't select it again
                      (ExecutionOutput
                        (GroundedSchema "scm: ghost-update-rule-strength")
                        ; Note: This is generated by psi-rule-set-alias!
                        (List (Concept (string-append psi-prefix-str RULENAME))
                              (Number 0)))
                      (list))
                  ; Set the current topic
                  (Put (State ghost-curr-topic (Variable "$x"))
                       rule-topic)))))

; ----------
(define (process-goal GOAL)
"
  Go through each of the goals, including the shared ones.
"
  (if (and (null? GOAL) (null? shared-goals))
      (begin
        (cog-logger-warn ghost-logger
          "Did you forget to link a goal to the rule?")
        (list (cons (ghost-prefix "Default Goal") .9)))
      ; The shared goals will be overwritten if the same goal is given
      ; to this rule
      (append GOAL
        (remove (lambda (sg) (any (lambda (g) (equal? (car sg) (car g))) GOAL))
                shared-goals))))

; ----------
(define (process-type TYPE)
"
  Figure out what the type of the rule is, generate the needed atomese, and
  return the type as a StringValue.
"
  (cond ((or (equal? #\u TYPE)
             (equal? #\s TYPE)
             (equal? #\? TYPE))
         (list '() '() strval-responder))
        ((equal? #\r TYPE)
         (list '() '() strval-random-gambit))
        ((equal? #\t TYPE)
         (list '() '() strval-gambit))
        ; For rejoinders, put the condition (the last rule executed is
        ; the parent of this rejoinder) in the pattern of the rule
        (else (let ((var (Variable (gen-var "GHOST-rule" #f)))
                    (lv (get-rejoinder-level TYPE)))
          (list
            (list (TypedVariable var (Type "ConceptNode")))
            (list (State ghost-last-executed var)
                  (Equal var
                    (Concept (string-append psi-prefix-str
                      (last (list-ref rule-lists (- lv 1)))))))
            strval-rejoinder)))))

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
  (define (add-to-rule-lists LV RULE)
    (if (<= (length rule-lists) LV)
        (set! rule-lists (append rule-lists (list (list RULE))))
        (list-set! rule-lists LV
          (append (list-ref rule-lists LV) (list RULE)))))

  ; First of all, make sure the topic is set
  ; so that it can be used when we are processing the action
  (if (null? rule-topic)
      (set! rule-topic (create-topic "Default Topic")))

  ; Reset the list of local variables
  (set! pat-vars '())

  ; Reset the rule-lists if we're looking at a new responder/gambit
  (if (or (equal? #\u TYPE) (equal? #\s TYPE) (equal? #\? TYPE)
          (equal? #\r TYPE) (equal? #\t TYPE))
      (set! rule-lists '()))

  (let* (; Label the rule with NAME, if given, generate one otherwise
         (rule-name (if (string-null? NAME)
                        (string-append "GHOST-rule-" (random-string 36))
                        NAME))
         (proc-type (process-type TYPE))
         (ordered-terms (order-terms PATTERN))
         (proc-terms (process-pattern-terms ordered-terms))
         (vars (append atomese-variable-template
                       (list-ref proc-terms 0)
                       (list-ref proc-type 0)))
         (conds (append atomese-condition-template
                        (list-ref proc-terms 1)
                        (list-ref proc-type 1)))
         (type (list-ref proc-type 2))
         (action (process-action ACTION rule-name))
         (goals (process-goal GOAL)))

        (cog-logger-debug ghost-logger "Context: ~a" ordered-terms)
        (cog-logger-debug ghost-logger "Procedure: ~a" ACTION)
        (cog-logger-debug ghost-logger "Goal: ~a" goals)

        (map (lambda (rule)
               ; Label the rule
               (psi-rule-set-alias! rule rule-name)
               ; Set the type
               (cog-set-value! rule ghost-rule-type type)
               ; Associate it with its topic
               (Inheritance rule rule-topic)
               ; Then finally add to the rule-lists
               (cond ((or (equal? type strval-responder)
                          (equal? type strval-random-gambit)
                          (equal? type strval-gambit))
                      (add-to-rule-lists 0 rule-name))
                     ((equal? type strval-rejoinder)
                      (add-to-rule-lists
                        (get-rejoinder-level TYPE) rule-name)))
               ; Return
               rule)
             (map (lambda (goal)
                    ; Create the rule(s)
                    (psi-rule
                      (list (Satisfaction (VariableList vars) (And conds)))
                      action
                      (psi-goal (car goal))
                      (stv (cdr goal) .9)
                      ghost-component))
                  goals))))

; ----------
(define (create-concept NAME MEMBERS)
"
  Create named concepts with explicit membership lists.

  NAME is the name of the concept, and the rest is the list of words
  and/or concepts that will be considered as the members of the concept.
"
  (map (lambda (m) (Reference m (Concept NAME)))
       (terms-to-atomese MEMBERS)))

(define (create-shared-goal GOAL)
"
  Create a topic level goal that will be shared among the rules under the
  same topic.
"
  (set! shared-goals GOAL))

(define*-public (create-topic TOPIC-NAME
                #:optional (FEATURES (list)) (KEYWORDS (list)))
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

  ; Reset the topic-level goals
  (set! shared-goals '())

  ; The set of features associate with the topic
  ; The features will be stored as "values"
  (cog-set-value! rule-topic ghost-topic-feature
    (apply LinkValue (map (lambda (f) (StringValue f)) FEATURES)))

  ; The set of keywords associate with the topic
  (for-each (lambda (kw) (Member kw rule-topic)) (terms-to-atomese KEYWORDS))

  rule-topic)

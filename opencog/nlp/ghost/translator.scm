;; Shared variables for all terms
(define atomese-variable-template
  (list (TypedVariable (Variable "$S") (Type "SentenceNode"))
        (TypedVariable (Variable "$P") (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template
  (list (Parse (Variable "$P") (Variable "$S"))
        (State ghost-curr-proc (Variable "$S"))))

(define (order-terms TERMS)
  "Order the terms in the intended order, and insert wildcards into
   appropriate positions of the sequence.
   No operation is needed if the pattern is supposed to be matched in
   any order.
   Finally make sure there is no meaningless consecutive
   wildcard/variable in the term list we are going to return, as that
   may cause a glob to be grounded incorrectly."
  (define (merge-wildcard INT1 INT2)
    (cons (max (car INT1) (car INT2))
          (cond ((or (negative? (cdr INT1)) (negative? (cdr INT2))) -1)
                (else (max (cdr INT1) (cdr INT2))))))
  (let* ((as (cons 'anchor-start "<"))
         (ae (cons 'anchor-end ">"))
         (wc (cons 'wildcard (cons 0 -1)))
         (unordered? (any (lambda (t) (equal? 'unordered-matching (car t))) TERMS))
         ; It's possible that the sequence does not having any word or concept etc,
         ; e.g. a rule may just be about not having either one of these words
         (empty-seq? (every (lambda (t) (equal? 'negation (car t))) TERMS))
         (start-anchor? (any (lambda (t) (equal? as t)) TERMS))
         (end-anchor? (any (lambda (t) (equal? ae t)) TERMS))
         (start (if start-anchor? (cdr (member as TERMS)) (list wc)))
         (end (if end-anchor?
                  (take-while (lambda (t) (not (equal? ae t))) TERMS)
                  (list wc)))
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

(define (process-pattern-terms TERMS)
  "Generate the atomese (i.e. the variable declaration and the pattern)
   for each of the TERMS."
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
  (if (equal? (length lemma-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              lemma-seq)))
      (MemberLink (List lemma-seq) ghost-no-constant))

  (list vars conds))

(define (process-action ACTION)
  "Convert ACTION into atomese."
  (define reuse #f)
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
              ; The grounding of a variable in original words
              ((equal? 'get_wvar (car n))
               (list-ref pat-vars (cdr n)))
              ; The grounding of a variable in lemmas
              ((equal? 'get_lvar (car n))
               (get-var-lemmas (list-ref pat-vars (cdr n))))
              ; Get the value of a user variable
              ((equal? 'get_uvar (car n)) (get-user-variable (cdr n)))
              ; Assign a value to a user variable
              ((equal? 'assign_uvar (car n))
               (set-user-variable (cadr n) (car (to-atomese (cddr n)))))
              ; A function call
              ((equal? 'function (car n))
               (if (equal? "reuse" (cadr n)) (set! reuse #t))
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
  (True (if reuse action-atomese
                  (list (ExecutionOutput
                          (GroundedSchema "scm: ghost-execute-action")
                          (List action-atomese))
                        (Put (State ghost-curr-topic (Variable "$x"))
                             rule-topic)))))

(define (process-goal GOAL)
  "Go through each of the goals, including the shared ones."
  (if (null? GOAL)
      (list (cons (ghost-prefix "Default Goal") 0.9))
      ; The shared goals will be overwritten if the same goal is specified
      ; to this rule
      (append GOAL
        (remove (lambda (sg) (any (lambda (g) (equal? (car sg) (car g))) GOAL))
                shared-goals))))

(define (create-rule PATTERN ACTION GOAL NAME TYPE)
  "Top level translation function. PATTERN, ACTION, and GOAL are the basic
   components of a psi-rule, correspond to context, procedure, and goal
   respectively. NAME is like a label of a rule, so that one can reference
   this rule by using it. TYPE is a grouping idea from ChatScript, e.g.
   responders, rejoinders, gambits etc."
  (catch #t
    (lambda ()
      ; First of all, make sure the topic is set
      (if (null? rule-topic)
          (set! rule-topic (create-topic "Default Topic" '())))
      (let* ((ordered-terms (order-terms PATTERN))
             (proc-terms (process-pattern-terms ordered-terms))
             (vars (append atomese-variable-template (list-ref proc-terms 0)))
             (conds (append atomese-condition-template (list-ref proc-terms 1)))
             (action (process-action ACTION))
             (goals (process-goal GOAL)))
            ; Reset the list of local variables
            (set! pat-vars '())
            ; Reset the rule-lists if we're looking at a new responder/gambit
            (if (or (equal? #\u TYPE) (equal? #\s TYPE) (equal? #\? TYPE)
                    (equal? #\r TYPE) (equal? #\t TYPE))
                (set! rule-lists '()))
            (cog-logger-debug ghost-logger "Context: ~a" ordered-terms)
            (cog-logger-debug ghost-logger "Procedure: ~a" ACTION)
            (cog-logger-debug ghost-logger "Goal: ~a" goals)
            (map (lambda (rule)
                   ; Label the rule(s), if needed
                   (if (string-null? NAME) rule (psi-rule-set-alias rule NAME))
                   ; Then check the rule type
                   ; TODO: Give it a rank
                   (cond ; For responders
                         ((or (equal? #\u TYPE)
                              (equal? #\s TYPE)
                              (equal? #\? TYPE))
                          (if (null? rule-lists)
                              (set! rule-lists (list (list rule)))
                              (list-set! rule-lists 0
                                (append (list-ref rule-lists 0) (list rule)))))
                         ; For gambits
                         ((or (equal? #\r TYPE)
                              (equal? #\t TYPE))
                          (if (null? rule-lists)
                              (set! rule-lists (list (list rule)))
                              (list-set! rule-lists 0
                                (append (list-ref rule-lists 0) (list rule)))))
                         ; For rejoinders
                         ; Rejoinders can be nested, a = level 1, b = level 2... etc
                         (else (let ((level (- (char->integer TYPE) 96)))
                           ; Store this rejoinder-relationship with each of its
                           ; parents in the atomspace
                           (map (lambda (parent-rule)
                                  (Evaluation (Predicate (ghost-prefix "rejoinder"))
                                    (List parent-rule rule)))
                                (list-ref rule-lists (- level 1)))
                           ; Update the rule-lists
                           (if (<= (length rule-lists) level)
                             (set! rule-lists (append rule-lists (list (list rule))))
                             (list-set! rule-lists level
                               (append (list-ref rule-lists level) (list rule)))))))
                   ; Return
                   rule)
                 (map (lambda (goal)
                        ; Create the rule(s)
                        (psi-rule
                          (list (Satisfaction (VariableList vars) (And conds)))
                          action
                          (psi-goal (car goal))
                          (stv (cdr goal) .9)
                          rule-topic))
                      goals))))
    (lambda (key . parameters)
      (if (not (equal? key 'FeatureNotSupported))
          (cog-logger-error ghost-logger "~a: ~a" key parameters)))))

(define (create-concept NAME MEMBERS)
  "Create named concepts with explicit membership lists.
   The first argument is the name of the concept, and the rest is the
   list of words and/or concepts that will be considered as the members
   of the concept."
  (map (lambda (m) (Reference m (Concept NAME)))
       (terms-to-atomese MEMBERS)))

(define (create-shared-goal GOAL)
  "Create a topic level goal that will be shared among the rules under the
   same topic."
  (set! shared-goals GOAL))

(define-public (create-topic TOPIC-NAME KEYWORDS)
"
  create-topic TOPIC-NAME

  Creates a psi-demand named as TOPIC-NAME, sets the rule-topic to be it
  and returns ConceptNode that represent the topic(aka demand).
"
  ; NOTE:The intention is to follow chatscript like authoring approach. Once a
  ; topic is created, then the rules that are added after that will be under
  ; that topic.

  ; TODO:
  ; 1. Should this be a skipped demand, so as to separate the dialogue loop
  ; be independent of the psi-loop? Or, is it better to resturcture openpsi to
  ; allow as many loops as possilbe as that might be required for the DMT
  ; implementation?
  ; 2. Should the weight be accessable? Specially if the execution graph is
  ; separate from the content, thus allowing learing, why?

  ; A topic will be defined when loading a (topic) file
  (set! rule-topic (psi-demand (ghost-prefix TOPIC-NAME)))
  (Inheritance rule-topic ghost-topic)

  ; Reset the topic-level goals
  (set! shared-goals '())

  ; The set of keywords associate with the topic
  (for-each (lambda (kw) (Member kw rule-topic)) (terms-to-atomese KEYWORDS))

  rule-topic)

;; Shared variables for all terms
(define atomese-variable-template
  (list (TypedVariable (Variable "$S") (Type "SentenceNode"))
        (TypedVariable (Variable "$P") (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template
  (list (Parse (Variable "$P") (Variable "$S"))
        (State ghost-anchor (Variable "$S"))))

(define (order-terms TERMS)
  "Order the terms in the intended order, and insert wildcards into
   appropriate positions of the sequence."
  (let* ((as (cons 'anchor-start "<"))
         (ae (cons 'anchor-end ">"))
         (wc (cons 'wildcard (cons 0 -1)))
         (start-anchor? (any (lambda (t) (equal? as t)) TERMS))
         (end-anchor? (any (lambda (t) (equal? ae t)) TERMS))
         (start (if start-anchor? (cdr (member as TERMS)) (list wc)))
         (end (if end-anchor?
                  (take-while (lambda (t) (not (equal? ae t))) TERMS)
                  (list wc))))
        (cond ((and start-anchor? end-anchor?)
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
              ; If there is no anchor, the main-seq should start and
              ; end with a wildcard
              (else (append (list wc) TERMS (list wc))))))

(define (preprocess-terms TERMS)
  "Make sure there is no meaningless wildcard/variable in TERMS,
   which may cause problems in rule matching or variable grounding."
  (define (merge-wildcard INT1 INT2)
    (cons (max (car INT1) (car INT2))
          (cond ((or (negative? (cdr INT1)) (negative? (cdr INT2))) -1)
                (else (max (cdr INT1) (cdr INT2))))))
  (fold-right (lambda (term prev)
    (cond ; If there are two consecutive wildcards, e.g.
          ; (wildcard 0 . -1) (wildcard 3. 5)
          ; merge them
          ((and (equal? 'wildcard (car term))
                (equal? 'wildcard (car (first prev))))
           (cons (cons 'wildcard
                       (merge-wildcard (cdr term) (cdr (first prev))))
                 (cdr prev)))
          ; If we have a variable that matches to "zero or more"
          ; follow by a wildcard, e.g.
          ; (variable (wildcard 0 . -1)) (wildcard 3 . 6)
          ; return the variable
          ((and (equal? 'variable (car term))
                (equal? (cadr term) (cons 'wildcard (cons 0 -1)))
                (equal? 'wildcard (car (first prev))))
           (cons term (cdr prev)))
          ; Otherwise accept and append it to the list
          (else (cons term prev))))
    (list (last TERMS))
    (list-head TERMS (- (length TERMS) 1))))

(define (process-pattern-terms TERMS)
  "Generate the atomese (i.e. the variable declaration and the pattern)
   for each of the TERMS."
  (define (process terms)
    (define vars '())
    (define conds '())
    (define word-seq '())
    (define lemma-seq '())
    (define is-unordered? #f)
    (define (update-lists t)
      (set! vars (append vars (list-ref t 0)))
      (set! conds (append conds (list-ref t 1)))
      (set! word-seq (append word-seq (list-ref t 2)))
      (set! lemma-seq (append lemma-seq (list-ref t 3))))
    (for-each (lambda (t)
      (cond ((equal? 'unordered-matching (car t))
             (let ((terms (process (cdr t))))
                  (update-lists terms)
                  (set! is-unordered? #t)))
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
            ((equal? 'negation (car t))
             (update-lists (negation (cdr t))))
            ((equal? 'wildcard (car t))
             (update-lists (wildcard (cadr t) (cddr t))))
            ((equal? 'variable (car t))
             (let ((terms (process (cdr t))))
                  (update-lists terms)
                  (set! pat-vars (append pat-vars (list-ref terms 2)))))
            ((equal? 'uvar_exist (car t))
             (set! conds (append conds (list (uvar-exist? (cdr t))))))
            ((equal? 'uvar_equal (car t))
             (set! conds (append conds (list (uvar-equal? (cadr t) (caddr t))))))
            ((equal? 'function (car t))
             (set! conds (append conds
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
            (else (feature-not-supported (car t) (cdr t)))))
      terms)
    (list vars conds word-seq lemma-seq is-unordered?))
  ; Start the processing
  (define proc-terms (process TERMS))
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  ; (list-ref proc-terms 3) is the lemma-seq
  (if (equal? (length (list-ref proc-terms 3))
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              (list-ref proc-terms 3))))
      (begin (MemberLink (List (list-ref proc-terms 3)) ghost-no-constant)
             (MemberLink (Set (list-ref proc-terms 3)) ghost-no-constant)))
  proc-terms)

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
              ((not (null? choices))
               (let ((ac (action-choices choices)))
                    (set! choices '())
                    (list ac)))
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
              (else (Word (cdr n)))))
        actions)
      (if (null? choices)
          '()
          (list (action-choices choices)))))
  (define action-atomese (to-atomese (cdar ACTION)))
  (cog-logger-debug ghost-logger "action: ~a" ACTION)
  (True (if reuse action-atomese
                  (ExecutionOutput (GroundedSchema "scm: ghost-execute-action")
                                   (List action-atomese)))))

(define (process-goal GOAL)
  "Go through each of the goals, including the shared ones."
  (if (null? GOAL)
      (list (cons (ghost-prefix "Default Goal") 0.9))
      ; The shared goals will be overwritten if the same goal is specified
      ; to this rule
      (append GOAL
        (remove (lambda (sg) (any (lambda (g) (equal? (car sg) (car g))) GOAL))
                shared-goals))))

(define (create-rule PATTERN ACTION GOAL TOPIC NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (cog-logger-debug "In create-rule\nPATTERN = ~a\nACTION = ~a" PATTERN ACTION)
  (let* ((ordered-terms (order-terms PATTERN))
         (preproc-terms (preprocess-terms ordered-terms))
         (proc-terms (process-pattern-terms preproc-terms))
         (vars (append atomese-variable-template (list-ref proc-terms 0)))
         (conds (append atomese-condition-template (list-ref proc-terms 1)))
         (is-unordered? (list-ref proc-terms 4))
         (words (if is-unordered?
           (Evaluation ghost-word-set
             (List (Variable "$S") (Set (list-ref proc-terms 2))))
           (Evaluation ghost-word-seq
             (List (Variable "$S") (List (list-ref proc-terms 2))))))
         (lemmas (if is-unordered?
           (Evaluation ghost-lemma-set
             (List (Variable "$S") (Set (list-ref proc-terms 3))))
           (Evaluation ghost-lemma-seq
             (List (Variable "$S") (List (list-ref proc-terms 3))))))
         (rule (map (lambda (goal)
                      (psi-rule
                        (list (Satisfaction (VariableList vars)
                                            (And words lemmas conds)))
                        (process-action ACTION)
                        (psi-goal (car goal))
                        (stv (cdr goal) .9)
                        (if (null? TOPIC) default-topic TOPIC)
                        NAME))
                    (process-goal GOAL))))
        (set! pat-vars '())
        (cog-logger-debug ghost-logger "ordered-terms: ~a" ordered-terms)
        (cog-logger-debug ghost-logger "preproc-terms: ~a" preproc-terms)
        (cog-logger-debug ghost-logger "psi-rule: ~a" rule)
        rule))

; ----------
; Topic
; ----------
(define default-topic '())

(define-public (create-topic TOPIC-NAME)
"
  create-topic TOPIC-NAME

  Creates a psi-demand named as TOPIC-NAME, sets the default-topic to be it
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

  (set! default-topic (psi-demand TOPIC-NAME))
  default-topic)

; This is the default topic.
(create-topic "Yakking")

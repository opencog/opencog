;; GHOST DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog logger)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
             (opencog eva-behavior)
             (srfi srfi-1)
             (rnrs io ports)
             (ice-9 popen)
             (ice-9 optargs))

; Keep a record of the variables, if any, found in the pattern of a rule
; TODO: Move it to process-pattern-terms?
(define pat-vars '())

; Keep a record of the groundings of variables that authors defined
(define var-grd-words '())
(define var-grd-lemmas '())

; Keep a record of the value assigned to the user variables that authors defined
(define uvars '())

; Keep a record of the lemmas we have seen
(define lemma-alist '())

; For unit test
; Use RelEx server to get the lemma, by default
; Otherwise, use WordNet CLI
(define test-get-lemma #f)

; Define the logger for GHOST
(define ghost-logger (cog-new-logger))
(cog-logger-set-level! ghost-logger "info")
(cog-logger-set-stdout! ghost-logger #t)

(define-public (ghost-prefix STR) (string-append "GHOST: " STR))
(define (ghost-var-word NUM)
  (Variable (ghost-prefix
    (string-append "variable-word-" (number->string NUM)))))
(define (ghost-var-lemma NUM)
  (Variable (ghost-prefix
    (string-append "variable-lemma-" (number->string NUM)))))
(define (ghost-uvar STR)
  (Variable (ghost-prefix (string-append "user-variable-" STR))))
(define ghost-anchor (Anchor (ghost-prefix "Currently Processing")))
(define ghost-no-constant (Anchor (ghost-prefix "No constant terms")))
(define ghost-word-seq (Predicate (ghost-prefix "Word Sequence")))
(define ghost-word-set (Predicate (ghost-prefix "Word Set")))
(define ghost-lemma-seq (Predicate (ghost-prefix "Lemma Sequence")))
(define ghost-lemma-set (Predicate (ghost-prefix "Lemma Set")))

; For features that are not currently supported
(define (feature-not-supported NAME VAL)
  "Notify the user that a particular is not currently supported."
  (cog-logger-error ghost-logger "Feature not supported: \"~a ~a\"" NAME VAL))

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
                  (set! conds (append conds
                    (list (variable (length pat-vars)
                                    (list-ref terms 2)))))
                  (set! pat-vars (append pat-vars (list-ref terms 2)))))
            ((equal? 'uvar_exist (car t))
             (set! conds (append conds (list (uvar-exist? (cdr t))))))
            ((equal? 'uvar_equal (car t))
             (set! conds (append conds (list (uvar-equal? (cadr t) (caddr t))))))
            ((equal? 'function (car t))
             (set! conds (append conds
               (list (context-function (cadr t)
                 (map (lambda (a)
                   (cond ((equal? 'get_wvar (car a)) (get-var-words (cdr a)))
                         ((equal? 'get_lvar (car a)) (get-var-lemmas (cdr a)))
                         ((equal? 'get_uvar (car a)) (get-user-variable (cdr a)))
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

(define-public (ghost-execute-action ACTION)
  "Say the text and update the internal state."
  (define (extract-txt action)
    ; TODO: Right now it is extracting text only, but should be extended
    ; to support actions that contains are not text as well.
    (string-join (append-map
      (lambda (a)
        (cond ((cog-link? a)
               (list (extract-txt a)))
              ((equal? 'WordNode (cog-type a))
               (list (cog-name a)))
              ; TODO: things other than text
              (else '())))
        (cog-outgoing-set action))))
  (define txt (extract-txt ACTION))
  ; Is there anything to say?
  (if (not (string-null? (string-trim txt)))
      (begin (cog-logger-info ghost-logger "Say: \"~a\"" txt)
             (cog-execute! (Put (DefinedPredicate "Say") (Node txt)))))
  ; Reset the state
  (State ghost-anchor (Concept "Default State")))

(define-public (ghost-pick-action ACTIONS)
  "Pick one of the ACTIONS randomly."
  (define os (cog-outgoing-set ACTIONS))
  (list-ref os (random (length os) (random-state-from-platform))))

(Define
  (DefinedSchema (ghost-prefix "Pick and Execute Action"))
  (Lambda (Variable "$x")
          (ExecutionOutput (GroundedSchema "scm: ghost-pick&execute-action")
                           (List (Variable "$x")))))

(Define
  (DefinedSchema (ghost-prefix "Execute Action"))
  (Lambda (Variable "$x")
          (ExecutionOutput (GroundedSchema "scm: ghost-execute-action")
                           (List (Variable "$x")))))

(define (process-action ACTION)
  "Iterate through each of the ACTIONS and convert them into atomese."
  (define (to-atomese actions)
    (define choices '())
    (append
      ; Iterate through the output word-by-word
      (map (lambda (n)
        (cond ; The grounding of a variable in original words
              ((equal? 'get_wvar (car n)) (get-var-words (cdr n)))
              ; The grounding of a variable in lemmas
              ((equal? 'get_lvar (car n)) (get-var-lemmas (cdr n)))
              ; Get the value of a user variable
              ((equal? 'get_uvar (car n)) (get-user-variable (cdr n)))
              ; Assign a value to a user variable
              ((equal? 'assign_uvar (car n))
               (set-user-variable (cadr n) (car (to-atomese (cddr n)))))
              ; A function call
              ((equal? 'function (car n))
               (action-function (cadr n) (to-atomese (cddr n))))
              ; Gather all the action choices, i.e. a list of actions
              ; available but only one of them will be executed
              ((equal? 'action-choices (car n))
               (set! choices (append choices (list (List (to-atomese (cdr n))))))
               '())
              (else (Word (cdr n)))))
        actions)
      (if (null? choices)
          '()
          (list (action-choices choices)))))
  (cog-logger-debug ghost-logger "action: ~a" ACTION)
  (True (Put (DefinedSchema (ghost-prefix "Execute Action"))
             (List (to-atomese (cdar ACTION))))))

(define* (create-rule PATTERN ACTION #:optional (TOPIC default-topic) NAME)
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
         (action (process-action ACTION))
         (psi-rule (psi-rule-nocheck
                     (list (Satisfaction (VariableList vars)
                                         (And words lemmas conds)))
                     action
                     (True)
                     (stv .9 .9)
                     TOPIC
                     NAME)))
        (cog-logger-debug ghost-logger "ordered-terms: ~a" ordered-terms)
        (cog-logger-debug ghost-logger "preproc-terms: ~a" preproc-terms)
        (cog-logger-debug ghost-logger "psi-rule: ~a" psi-rule)
        (set! pat-vars '())  ; Reset pat-vars
        psi-rule))

(define (sent-get-word-seqs SENT)
  "Get the words (original and lemma) associate with SENT.
   It also creates an EvaluationLink linking the
   SENT with the word-list and lemma-list."
  (define (get-seq TYPE)
    (append-map
      (lambda (w)
        ; Ignore LEFT-WALL and punctuations
        (if (or (string-prefix? "LEFT-WALL" (cog-name w))
                (word-inst-match-pos? w "punctuation")
                (null? (cog-chase-link TYPE 'WordNode w)))
            '()
            ; For proper names, e.g. Jessica Henwick,
            ; RelEx converts them into a single WordNode, e.g.
            ; (WordNode "Jessica_Henwick"). Codes below try to
            ; split it into two WordNodes, "Jessica" and "Henwick",
            ; so that the matcher will be able to find the rules
            (let* ((wn (car (cog-chase-link TYPE 'WordNode w)))
                   (name (cog-name wn)))
              (if (integer? (string-index name #\_))
                  (map Word (string-split name  #\_))
                  (list wn)))))
      (car (sent-get-words-in-order SENT))))
  (let* ((wseq (get-seq 'ReferenceLink))
         (word-seq (List wseq))
         (word-set (Set wseq))
         (lseq (get-seq 'LemmaLink))
         (lemma-seq (List lseq))
         (lemma-set (Set lseq)))
        ; These EvaluationLinks will be used in the matching process
        (Evaluation ghost-word-seq (List SENT word-seq))
        (Evaluation ghost-word-set (List SENT word-set))
        (Evaluation ghost-lemma-seq (List SENT lemma-seq))
        (Evaluation ghost-lemma-set (List SENT lemma-set))
        (list word-seq word-set lemma-seq lemma-set)))

(define (get-lemma-from-relex WORD)
  "Get the lemma of WORD via the RelEx server."
  (relex-parse WORD)
  (let* ((sent (car (get-new-parsed-sentences)))
         (word-inst (cadar (sent-get-words-in-order sent)))
         (lemma (cog-name (car (cog-chase-link 'LemmaLink 'WordNode word-inst)))))
    (release-new-parsed-sents)
    (if (equal? (string-downcase WORD) lemma)
        WORD
        lemma)))

(define (get-lemma-from-wn WORD)
  "A hacky way to quickly find the lemma of a word using WordNet,
   for unit test only."
  (let* ((cmd-string
           (string-append "wn " WORD " | grep \"Information available for .\\+\""))
         (port (open-input-pipe cmd-string))
         (lemma ""))
    (do ((line (get-line port) (get-line port)))
        ((eof-object? line))
      (let ((l (car (last-pair (string-split line #\ )))))
        (if (not (equal? (string-downcase WORD) l))
          (set! lemma l))))
    (close-pipe port)
    (if (string-null? lemma) WORD lemma)))

(define (get-lemma WORD)
  "Get the lemma of WORD."
  (define seen-lemma (assoc-ref lemma-alist WORD))
  (if (equal? #f seen-lemma)
      (let ((lemma
        (if test-get-lemma
            (get-lemma-from-wn WORD)
            (get-lemma-from-relex WORD))))
        (set! lemma-alist (assoc-set! lemma-alist WORD lemma))
        lemma)
      seen-lemma))

(define-public (ghost-get-lemma . WORD)
  "Get the lemma of WORD, where WORD can be one or more WordNodes."
  (List (map Word (map get-lemma (map cog-name WORD)))))

(define (is-lemma? WORD)
  "Check if WORD is a lemma."
  (equal? WORD (get-lemma WORD)))

(define (get-members CONCEPT)
  "Get the members of a concept. VariableNodes will be ignored, and
   recursive calls will be made in case there are nested concepts."
  (append-map
    (lambda (g)
      (cond ((eq? 'ConceptNode (cog-type g)) (get-members g))
            ((eq? 'VariableNode (cog-type g)) '())
            (else (list g))))
    (cog-outgoing-set
      (cog-execute! (Get (Reference (Variable "$x") CONCEPT))))))

(define (is-member? GRD LST)
  "Check if GRD (the grounding of a glob) is a member of LST,
   where LST may contain WordNodes, LemmaNodes, and PhraseNodes."
  (let* ((raw-txt (string-join (map cog-name GRD)))
         (lemma-txt (car (map (lambda (w) (get-lemma (cog-name w))) GRD))))
    (any (lambda (t)
           (or (and (eq? 'WordNode (cog-type t))
                    (equal? raw-txt (cog-name t)))
               (and (eq? 'LemmaNode (cog-type t))
                    (equal? lemma-txt (cog-name t)))
               (and (eq? 'PhraseNode (cog-type t))
                    (equal? raw-txt (cog-name t)))))
         LST)))

(define-public (ghost-concept? CONCEPT . GRD)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept."
  (cog-logger-debug ghost-logger
    "In ghost-concept? CONCEPT: ~aGRD: ~a" CONCEPT GRD)
  (if (is-member? GRD (get-members CONCEPT))
      (stv 1 1)
      (stv 0 1)))

(define-public (ghost-choices? CHOICES . GRD)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices."
  (cog-logger-debug ghost-logger
    "In ghost-choices? CHOICES: ~aGRD: ~a" CHOICES GRD)
  (let* ((chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (is-member? GRD (append chs cpts))
            (stv 1 1)
            (stv 0 1))))

(define (text-contains? RTXT LTXT TERM)
  "Check if either RTXT (raw) or LTXT (lemma) contains TERM."
  (define (contains? txt term)
    (not (equal? #f (regexp-exec
      (make-regexp (string-append "\\b" term "\\b") regexp/icase) txt))))
  (cond ((equal? 'WordNode (cog-type TERM))
         (contains? RTXT (cog-name TERM)))
        ((equal? 'LemmaNode (cog-type TERM))
         (contains? LTXT (cog-name TERM)))
        ((equal? 'PhraseNode (cog-type TERM))
         (contains? RTXT (cog-name TERM)))
        ((equal? 'ConceptNode (cog-type TERM))
         (any (lambda (t) (text-contains? RTXT LTXT t))
              (get-members TERM)))))

(define-public (ghost-negation? . TERMS)
  "Check if the input sentence has none of the terms specified."
  (let* ; Get the raw text input
        ((sent (car (cog-chase-link 'StateLink 'SentenceNode ghost-anchor)))
         (rtxt (cog-name (car (cog-chase-link 'ListLink 'Node sent))))
         (ltxt (string-join (map get-lemma (string-split rtxt #\sp)))))
        (if (any (lambda (t) (text-contains? rtxt ltxt t)) TERMS)
            (stv 0 1)
            (stv 1 1))))

(define (member-words STR)
  "Convert the member in the form of a string into an atom, which can
   either be a WordNode, LemmaNode, ConceptNode, or a PhraseNode."
  (let ((words (string-split STR #\sp)))
    (if (= 1 (length words))
        (let ((w (car words)))
             (cond ((string-prefix? "~" w) (Concept (substring w 1)))
                   ((is-lemma? w) (LemmaNode w))
                   (else (WordNode w))))
        (PhraseNode STR))))

(define (create-concept NAME MEMBERS)
  "Create named concepts with explicit membership lists.
   The first argument is the name of the concept, and the rest is the
   list of words and/or concepts that will be considered as the members
   of the concept."
  (append-map (lambda (m) (list (Reference (member-words m) (Concept NAME))))
              MEMBERS))

(define-public (ghost-get-var-words VAR)
  "Get the grounding of VAR, in original words."
  (define grd (assoc-ref var-grd-words VAR))
  (if (equal? grd #f)
      (List)
      grd))

(define-public (ghost-get-var-lemmas VAR)
  "Get the grounding of VAR, in lemmas."
  (define grd (assoc-ref var-grd-lemmas VAR))
  (if (equal? grd #f) (List) grd))

(define-public (ghost-get-user-variable UVAR)
  "Get the value stored for VAR."
  (define grd (assoc-ref uvars UVAR))
  (if (equal? grd #f) (List) grd))

(define-public (ghost-set-user-variable UVAR VAL)
  "Assign VAL to UVAR."
  (set! uvars (assoc-set! uvars UVAR VAL))
  (True))

(define-public (ghost-user-variable-exist? UVAR)
  "Check if UVAR has been defined."
  (if (equal? (assoc-ref uvars UVAR) #f)
      (stv 0 1)
      (stv 1 1)))

(define-public (ghost-user-variable-equal? UVAR VAL)
  "Check if the value of UVAR equals VAL."
  (if (equal? (assoc-ref uvars UVAR) VAL)
      (stv 1 1)
      (stv 0 1)))

(define-public (ghost-record-groundings WGRD LGRD)
  "Record the groundings of a variable, in both original words and lemmas.
   They will be referenced at the stage of evaluating the context of the
   psi-rules, or executing the action of the psi-rules."
  (cog-logger-debug ghost-logger "--- Recording groundings:\n~a\n~a" WGRD LGRD)
  (set! var-grd-words (assoc-set! var-grd-words (gar WGRD) (gdr WGRD)))
  (set! var-grd-lemmas (assoc-set! var-grd-lemmas (gar LGRD) (gdr LGRD)))
  (stv 1 1))

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

  (set! default-topic (psi-demand TOPIC-NAME 0.9))
  default-topic)

; This is the default topic.
(create-topic "Yakking")

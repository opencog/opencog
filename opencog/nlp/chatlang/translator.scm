;; ChatLang DSL for chat authoring rules
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

; For storing the groundings
(define globs-word '())
(define globs-lemma '())

; Keep a record of the variables, if any, found in the pattern of a rule
(define pat-vars '())

; For unit test
(define test-get-lemma #f)

(define-public (chatlang-prefix STR) (string-append "Chatlang: " STR))
(define chatlang-anchor (Anchor (chatlang-prefix "Currently Processing")))
(define chatlang-no-constant (Anchor (chatlang-prefix "No constant terms")))
(define chatlang-word-seq (Predicate (chatlang-prefix "Word Sequence")))
(define chatlang-lemma-seq (Predicate (chatlang-prefix "Lemma Sequence")))
(define chatlang-lemma-set (Predicate (chatlang-prefix "Lemma Set")))

;; Shared variables for all terms
(define atomese-variable-template (list (TypedVariable (Variable "$S")
                                                       (Type "SentenceNode"))
                                        (TypedVariable (Variable "$P")
                                                       (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template (list (Parse (Variable "$P")
                                                (Variable "$S"))
                                         (State chatlang-anchor
                                                (Variable "$S"))))

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
  (define vars '())
  (define globs '())
  (define conds '())
  (define glob-conds '())
  (define term-seq '())
  (for-each (lambda (t)
    (cond ((equal? 'lemma (car t))
           (let ((l (lemma (cdr t))))
                (set! vars (append vars (car l)))
                (set! conds (append conds (cdr l)))
                (set! term-seq
                  (append term-seq (list (Word (get-lemma (cdr t))))))))
          ((equal? 'word (car t))
           (let ((w (word (cdr t))))
                (set! vars (append vars (car w)))
                (set! conds (append conds (cdr w)))
                (set! term-seq
                  (append term-seq (list (Word (get-lemma (cdr t))))))))
          ((equal? 'phrase (car t))
           (let ((p (phrase (cdr t))))
                (set! vars (append vars (car p)))
                (set! conds (append conds (cdr p)))
                (set! term-seq (append term-seq
                  (map Word (map get-lemma (string-split (cdr t) #\sp)))))))
          ((equal? 'concept (car t))
           (let* ((v (choose-var-name))
                  (c (concept (cdr t) v))
                  (cl (concept (cdr t) v #t)))
                 (set! globs (append globs (car c)))
                 (set! conds (append conds (cdr c)))
                 (set! glob-conds (append glob-conds (cdr cl)))
                 (set! term-seq (append term-seq (list (Glob v))))))
          ((equal? 'choices (car t))
           (let* ((v (choose-var-name))
                  (c (choices (cdr t) v))
                  (cl (choices (cdr t) v #t)))
                 (set! globs (append globs (car c)))
                 (set! conds (append conds (cdr c)))
                 (set! glob-conds (append glob-conds (cdr cl)))
                 (set! term-seq (append term-seq (list (Glob v))))))
          ((equal? 'unordered-matching (car t))
           (let* ((v (choose-var-name))
                  (u (unordered-matching (cdr t) v)))
                 (set! vars (append vars
                   (filter (lambda (x) (equal? 'VariableNode (cog-type (gar x))))
                           (car u))))
                 (set! globs (append globs
                   (filter (lambda (x) (equal? 'GlobNode (cog-type (gar x))))
                           (car u))))
                 (set! conds (append conds (cdr u)))
                 (set! term-seq (append term-seq (list (Glob v))))))
          ((equal? 'negation (car t))
           (set! conds (append conds (cdr (negation (cdr t))))))
          ((equal? 'wildcard (car t))
           (let* ((v (choose-var-name))
                  (w (wildcard (cadr t) (cddr t) v)))
                 (set! globs (append globs (car w)))
                 (set! term-seq (append term-seq (list (Glob v))))))
          ((and (equal? 'variable (car t)) (equal? 'wildcard (caadr t)))
           (let* ((v (choose-var-name))
                  (w (wildcard (cadadr t) (cddadr t) v))
                  (glob (Glob v)))
                 (set! globs (append globs (car w)))
                 (set! term-seq (append term-seq (list glob)))
                 (set! pat-vars (append pat-vars (list glob)))))
          ((and (equal? 'variable (car t)) (equal? 'lemma (caadr t)))
           (let* ((v (choose-var-name))
                  (vl (var-lemma (cdadr t) v))
                  (glob (Glob v)))
                 (set! globs (append globs (car vl)))
                 (set! glob-conds (append glob-conds (cdr vl)))
                 (set! term-seq (append term-seq (list glob)))
                 (set! pat-vars (append pat-vars (list glob)))))
          ((and (equal? 'variable (car t)) (equal? 'concept (caadr t)))
           (let* ((v (choose-var-name))
                  (cl (concept (cdadr t) v #t))
                  (glob (Glob v)))
                 (set! globs (append globs (car cl)))
                 (set! glob-conds (append glob-conds (cdr cl)))
                 (set! term-seq (append term-seq (list (Glob v))))
                 (set! pat-vars (append pat-vars (list glob)))))
          ((and (equal? 'variable (car t)) (equal? 'choices (caadr t)))
           (let* ((v (choose-var-name))
                  (cl (choices (cdadr t) v #t))
                  (glob (Glob v)))
                 (set! globs (append globs (car cl)))
                 (set! glob-conds (append glob-conds (cdr cl)))
                 (set! term-seq (append term-seq (list (Glob v))))
                 (set! pat-vars (append pat-vars (list glob)))))))
    TERMS)
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  (if (equal? (length term-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              term-seq)))
    (MemberLink (List term-seq) chatlang-no-constant))
  (list vars globs conds glob-conds term-seq))

(define-public (ground-word GLOB)
  "Get the original words grounded for GLOB."
  (let ((gw (assoc-ref globs-word GLOB)))
       (if (equal? #f gw) (List) gw)))

(define-public (ground-lemma GLOB)
  "Get the lemmas grounded for GLOB."
  (let ((gl (assoc-ref globs-lemma GLOB)))
       (if (equal? #f gl) (List) gl)))

(define-public (chatlang-say WORDS)
  "Say the text and update the internal state."
  (define txt (string-join (append-map (lambda (n)
    (if (equal? 'ListLink (cog-type n))
        (map cog-name (cog-outgoing-set n))
        (list (cog-name n))))
    (cog-outgoing-set WORDS))))
  (cog-execute! (Put (DefinedPredicate "Say") (Node txt)))
  (State chatlang-anchor (Concept "Default State"))
  (True))

(Define
  (DefinedPredicate (chatlang-prefix "Say"))
  (Lambda (Variable "$x")
          (Evaluation (GroundedPredicate "scm: chatlang-say")
                      (List (Variable "$x")))))

(define-public (say TXT)
  "Say the text and clear the state."
  ; Replace the variables, if any, with the corresponding GlobNode
  (define txt-lst
    ; Iterate through the output word-by-word
    (map (lambda (n)
      (cond ; The grounding of a variable in original words
            ((not (equal? #f (string-match "'_[0-9]+" n)))
             (ExecutionOutput (GroundedSchema "scm: ground-word")
               (List (list-ref pat-vars (string->number (substring n 2))))))
            ; The grounding of a variable in lemmas
            ((not (equal? #f (string-match "_[0-9]+" n)))
             (ExecutionOutput (GroundedSchema "scm: ground-lemma")
               (List (list-ref pat-vars (string->number (substring n 1))))))
            ; A function call with no arguments
            ((not (equal? #f (string-match "\\^[a-zA-Z0-9_\\-\\(\\)]+" n)))
             (ExecutionOutput (GroundedSchema (string-append "scm: "
               (match:substring (string-match "[a-zA-Z0-9_\\-]+" n)))) (List)))
            (else (Word n))))
      (string-split TXT #\sp)))
  (True (Put (DefinedPredicate (chatlang-prefix "Say")) (List txt-lst))))

(define (process-action ACTION)
  "Process a single action -- converting it into atomese."
  (cond ((equal? 'say (car ACTION))
         (say (cdr ACTION)))))

(define-public (store-groundings SENT GRD)
  "Store the groundings, both original words and lemmas,
   for each of the GlobNode in the pattern.
   They will be referenced at the stage of evaluating the context
   of the psi-rules, or executing the action of the psi-rules."
  (let ((sent-word-seq (cog-outgoing-set (car (sent-get-word-seqs SENT))))
        (cnt 0))
       (for-each (lambda (g)
         (if (equal? 'ListLink (cog-type g))
             (if (equal? (gar g) (gadr g))
                 ; If the grounded value is the GlobNode itself,
                 ; that means the GlobNode is grounded to nothing
                 (begin
                   (set! globs-word (assoc-set! globs-word (gar g) (List)))
                   (set! globs-lemma (assoc-set! globs-lemma (gar g) (List))))
                 ; Store the GlobNode and the groundings
                 (begin
                   (set! globs-word (assoc-set! globs-word (gar g)
                     (List (take (drop sent-word-seq cnt)
                                 (length (cog-outgoing-set (gdr g)))))))
                   (set! globs-lemma (assoc-set! globs-lemma (gar g) (gdr g)))
                   (set! cnt (+ cnt (length (cog-outgoing-set (gdr g)))))))
             ; Move on if it's not a GlobNode
             (set! cnt (+ cnt 1))))
         (cog-outgoing-set GRD)))
  (True))

(define (generate-bind GLOB-DECL GLOB-COND TERM-SEQ)
  "Generate a BindLink that contains the TERM-SEQ and the
   restrictions on the GlobNode in the TERM-SEQ, if any."
  (Bind (VariableList GLOB-DECL
                      (TypedVariable (Variable "$S")
                                     (Type "SentenceNode")))
        (And GLOB-COND
             TERM-SEQ
             (State chatlang-anchor (Variable "$S")))
        (ExecutionOutput (GroundedSchema "scm: store-groundings")
                         (List (Variable "$S")
                               (List (map (lambda (x)
                                 (if (equal? 'GlobNode (cog-type x))
                                     (List (Quote x) (List x))
                                     x))
                                 (cog-outgoing-set (gddr TERM-SEQ))))))))

(define* (chat-rule PATTERN ACTION #:optional (TOPIC default-topic) NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((ordered-terms (order-terms PATTERN))
         (preproc-terms (preprocess-terms ordered-terms))
         (proc-terms (process-pattern-terms preproc-terms))
         (vars (append atomese-variable-template (list-ref proc-terms 0)))
         (globs (list-ref proc-terms 1))
         (conds (append atomese-condition-template (list-ref proc-terms 2)))
         (glob-conds (list-ref proc-terms 3))
         (term-seq (Evaluation chatlang-lemma-seq
                     (List (Variable "$S") (List (list-ref proc-terms 4)))))
         (action (process-action ACTION))
         (bindlink (generate-bind globs glob-conds term-seq))
         (psi-rule (psi-rule-nocheck
                     (list (Satisfaction (VariableList vars) (And conds)))
                     action
                     (True)
                     (stv .9 .9)
                     TOPIC
                     NAME)))
        (cog-logger-debug chatlang-logger "ordered-terms: ~a" ordered-terms)
        (cog-logger-debug chatlang-logger "preproc-terms: ~a" preproc-terms)
        (cog-logger-debug chatlang-logger "BindLink: ~a" bindlink)
        (cog-logger-debug chatlang-logger "psi-rule: ~a" psi-rule)
        ; Link both the newly generated BindLink and psi-rule together
        (Reference bindlink psi-rule)))

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
  (let* ((word-seq (List (get-seq 'ReferenceLink)))
         (lseq (get-seq 'LemmaLink))
         (lemma-seq (List lseq))
         (lemma-set (Set lseq)))
        ; These EvaluationLinks will be used in the matching process
        (Evaluation chatlang-word-seq (List SENT word-seq))
        (Evaluation chatlang-lemma-seq (List SENT lemma-seq))
        (Evaluation chatlang-lemma-set (List SENT lemma-set))
        (list word-seq lemma-seq lemma-set)))

(define (get-lemma-from-relex WORD)
  "Get the lemma of WORD via the RelEx server."
  (relex-parse WORD)
  (let* ((sent (car (get-new-parsed-sentences)))
         (word-inst (cadar (sent-get-words-in-order sent)))
         (lemma (car (cog-chase-link 'LemmaLink 'WordNode word-inst))))
    (release-new-parsed-sents)
    (if (equal? (string-downcase WORD) (cog-name lemma))
        WORD
        (cog-name lemma))))

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
  (if test-get-lemma
      (get-lemma-from-wn WORD)
      (get-lemma-from-relex WORD)))

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

(define (is-member? GLOB LST IN-LEMMA?)
  "Check if GLOB is a member of LST, where LST may contain
   WordNodes, LemmaNodes, and PhraseNodes. IN-LEMMA? is a flag
   to indicate whether the comparison should be done purely
   in lemmas."
  (if IN-LEMMA?
      (any (lambda (t)
        (equal? (string-join (map get-lemma (map cog-name GLOB)))
                (string-join (map get-lemma (string-split (cog-name t) #\sp)))))
        LST)
      (let* ((raw-txt (string-join (map cog-name
               (cog-outgoing-set (assoc-ref globs-word (car GLOB))))))
             (lemma-txt (string-join (map cog-name
               (cog-outgoing-set (assoc-ref globs-lemma (car GLOB)))))))
        (any (lambda (t)
               (or (and (eq? 'WordNode (cog-type t))
                        (equal? raw-txt (cog-name t)))
                   (and (eq? 'LemmaNode (cog-type t))
                        (equal? lemma-txt (cog-name t)))
                   (and (eq? 'PhraseNode (cog-type t))
                        (equal? raw-txt (cog-name t)))))
             LST))))

(define-public (chatlang-concept? CONCEPT . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept."
  (cog-logger-debug chatlang-logger
    "In chatlang-concept? CONCEPT: ~aGLOB: ~a" CONCEPT GLOB)
  (if (is-member? GLOB (get-members CONCEPT) #f)
      (stv 1 1)
      (stv 0 1)))

(define-public (chatlang-concept-in-lemma? CONCEPT . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept. All the comparison will be done in lemmas."
  (cog-logger-debug chatlang-logger
    "In chatlang-concept-in-lemma? CONCEPT: ~aGLOB: ~a" CONCEPT GLOB)
  (if (is-member? GLOB (get-members CONCEPT) #t)
      (stv 1 1)
      (stv 0 1)))

(define-public (chatlang-choices? CHOICES . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices."
  (cog-logger-debug chatlang-logger
    "In chatlang-choices? CHOICES: ~aGLOB: ~a" CHOICES GLOB)
  (let* ((chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (is-member? GLOB (append chs cpts) #f)
            (stv 1 1)
            (stv 0 1))))

(define-public (chatlang-choices-in-lemma? CHOICES . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices. All the comparison will be done in lemmas."
  (cog-logger-debug chatlang-logger
    "In chatlang-choices-in-lemma? CHOICES: ~aGLOB: ~a" CHOICES GLOB)
  (let* ((chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (is-member? GLOB (append chs cpts) #t)
            (stv 1 1)
            (stv 0 1))))

(define-public (chatlang-lemma? LEMMA . GLOB)
  "Check if the lemma of the value grounded for the GlobNode is LEMMA.
   For example if there is a variable \"_play\" in the pattern of a rule,
   it will be accepted if the value grounded is either play, plays, or
   played."
  (cog-logger-debug chatlang-logger
    "In chatlang-lemma? LEMMA: ~aGLOB: ~a" LEMMA GLOB)
  (if (is-member? GLOB (list LEMMA) #t)
    (stv 1 1)
    (stv 0 1)))

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

(define-public (chatlang-negation? . TERMS)
  "Check if the input sentence has none of the terms specified."
  (let* ; Get the raw text input
        ((sent (car (cog-chase-link 'StateLink 'SentenceNode chatlang-anchor)))
         (rtxt (cog-name (car (cog-chase-link 'ListLink 'Node sent))))
         (ltxt (string-join (map get-lemma (string-split rtxt #\sp)))))
        (if (any (lambda (t) (text-contains? rtxt ltxt t)) TERMS)
            (stv 0 1)
            (stv 1 1))))

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
; TODO: Move it to process-pattern-terms?
(define pat-vars '())

; For unit test
(define test-get-lemma #f)

(define-public (chatlang-prefix STR) (string-append "Chatlang: " STR))
(define chatlang-anchor (Anchor (chatlang-prefix "Currently Processing")))
(define chatlang-no-constant (Anchor (chatlang-prefix "No constant terms")))
(define chatlang-word-seq (Predicate (chatlang-prefix "Word Sequence")))
(define chatlang-word-set (Predicate (chatlang-prefix "Word Set")))
(define chatlang-lemma-seq (Predicate (chatlang-prefix "Lemma Sequence")))
(define chatlang-lemma-set (Predicate (chatlang-prefix "Lemma Set")))

;; Shared variables for all terms
(define atomese-variable-template
  (list (TypedVariable (Variable "$S") (Type "SentenceNode"))
        (TypedVariable (Variable "$P") (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template
  (list (Parse (Variable "$P") (Variable "$S"))
        (State chatlang-anchor (Variable "$S"))))

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

; JJJ TODO FIXME
; - use fold
; - don't do recursive process-pattern-terms calls, may generate
;   no-constant garbage
(define (process-pattern-terms TERMS)
  "Generate the atomese (i.e. the variable declaration and the pattern)
   for each of the TERMS."
  (define vars '())
  (define conds '())
  (define word-seq '())
  (define lemma-seq '())
  (define is-unordered? #f)
  (for-each (lambda (t)
    (cond ((equal? 'unordered-matching (car t))
           (let ((terms (process-pattern-terms (cdr t))))
                (set! vars (append vars (list-ref terms 0)))
                (set! conds (append conds (list-ref terms 1)))
                (set! word-seq (append word-seq (list-ref terms 2)))
                (set! lemma-seq (append lemma-seq (list-ref terms 3)))
                (set! is-unordered? #t)))
          ((equal? 'word (car t))
           (let ((w (word (cdr t))))
                (set! vars (append vars (list-ref w 0)))
                (set! conds (append conds (list-ref w 1)))
                (set! word-seq (append word-seq (list-ref w 2)))
                (set! lemma-seq (append lemma-seq (list-ref w 3)))))
          ((equal? 'lemma (car t))
           (let ((l (lemma (cdr t))))
                (set! vars (append vars (list-ref l 0)))
                (set! conds (append conds (list-ref l 1)))
                (set! word-seq (append word-seq (list-ref l 2)))
                (set! lemma-seq (append lemma-seq (list-ref l 3)))))
          ((equal? 'phrase (car t))
           (let ((p (phrase (cdr t))))
                (set! vars (append vars (list-ref p 0)))
                (set! conds (append conds (list-ref p 1)))
                (set! word-seq (append word-seq (list-ref p 2)))
                (set! lemma-seq (append lemma-seq (list-ref p 3)))))
          ((equal? 'concept (car t))
           (let ((c (concept (cdr t))))
                (set! vars (append vars (list-ref c 0)))
                (set! conds (append conds (list-ref c 1)))
                (set! word-seq (append word-seq (list-ref c 2)))
                (set! lemma-seq (append lemma-seq (list-ref c 3)))))
          ((equal? 'choices (car t))
           (let ((c (choices (cdr t))))
                (set! vars (append vars (list-ref c 0)))
                (set! conds (append conds (list-ref c 1)))
                (set! word-seq (append word-seq (list-ref c 2)))
                (set! lemma-seq (append lemma-seq (list-ref c 3)))))
          ((equal? 'negation (car t))
           ; Sure just do the same things like the above, not just for 1
           (set! conds (append conds (list-ref (negation (cdr t)) 1))))
          ((equal? 'wildcard (car t))
           (let ((w (wildcard (cadr t) (cddr t))))
                (set! vars (append vars (list-ref w 0)))
                (set! conds (append conds (list-ref w 1)))
                (set! word-seq (append word-seq (list-ref w 2)))
                (set! lemma-seq (append lemma-seq (list-ref w 3)))))
           ((equal? 'variable (car t))
            (let ((terms (process-pattern-terms (cdr t))))
                 (set! vars (append vars (list-ref terms 0)))
                 (set! conds (append conds
                                    (list-ref terms 1)
                                    (variable (list-ref terms 2))))
                 (set! word-seq (append word-seq (list-ref terms 2)))
                 (set! lemma-seq (append lemma-seq (list-ref terms 3)))
                 (set! pat-vars (append pat-vars (list-ref terms 2)))))))
    TERMS)
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  (if (equal? (length lemma-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              lemma-seq)))
      (begin (MemberLink (List lemma-seq) chatlang-no-constant)
             (MemberLink (Set lemma-seq) chatlang-no-constant)))
  (list vars conds word-seq lemma-seq is-unordered?))

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

(define (process-action ACTION)
  "Convert ACTION into atomese."
  ; Replace the variables, if any, with the corresponding GlobNode
  (define atomese
    ; Iterate through the output word-by-word
    (map (lambda (n)
      (cond ; The grounding of a variable in original words
            ((equal? 'get_wvar (car n))
             (ExecutionOutput (GroundedSchema "scm: ground-word")
               (List (list-ref pat-vars (cdr n)))))
            ; The grounding of a variable in lemmas
            ((equal? 'get_lvar (car n))
             (ExecutionOutput (GroundedSchema "scm: ground-lemma")
               (List (list-ref pat-vars (cdr n)))))
            ; A function call
            ((equal? 'function (car n))
             (ExecutionOutput (GroundedSchema (string-append "scm: " (cadr n)))
                              ; TODO: Use ConceptNode or?
                              (List (map Node (cddr n)))))
            (else (Word n))))
      ACTION))
  ; TODO: How about an action that only updates internal parameters?
  (True (Put (DefinedPredicate (chatlang-prefix "Say")) (List atomese))))

(define-public (record-groundings GLOB GRD)
  "Record the groundings of a variable/glob, in both original words
   and lemmas. They will be referenced at the stage of evaluating the
   context of the psi-rules, or executing the action of the psi-rules."
  (set! globs-word (assoc-set! globs-word GLOB (List GRD)))
  (set! globs-lemma (assoc-set! globs-lemma GLOB (List GED)))
  (True))

(define* (create-rule PATTERN ACTION #:optional (TOPIC default-topic) NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((ordered-terms (order-terms PATTERN))
         (preproc-terms (preprocess-terms ordered-terms))
         (proc-terms (process-pattern-terms preproc-terms))
         (vars (append atomese-variable-template (list-ref proc-terms 0)))
         (conds (append atomese-condition-template (list-ref proc-terms 1)))
         (is-unordered? (list-ref proc-terms 4))
         (words (if is-unordered?
           (Evaluation chatlang-word-set
             (List (Variable "$S") (Set (list-ref proc-terms 2))))
           (Evaluation chatlang-word-seq
             (List (Variable "$S") (List (list-ref proc-terms 2))))))
         (lemmas (if is-unordered?
           (Evaluation chatlang-lemma-set
             (List (Variable "$S") (Set (list-ref proc-terms 3))))
           (Evaluation chatlang-lemma-seq
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
        (cog-logger-debug chatlang-logger "ordered-terms: ~a" ordered-terms)
        (cog-logger-debug chatlang-logger "preproc-terms: ~a" preproc-terms)
        (cog-logger-debug chatlang-logger "psi-rule: ~a" psi-rule)
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
        (Evaluation chatlang-word-seq (List SENT word-seq))
        (Evaluation chatlang-word-set (List SENT word-set))
        (Evaluation chatlang-lemma-seq (List SENT lemma-seq))
        (Evaluation chatlang-lemma-set (List SENT lemma-set))
        (list word-seq word-set lemma-seq lemma-set)))

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

(define-public (chatlang-concept? CONCEPT . GRD)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept."
  (cog-logger-debug chatlang-logger
    "In chatlang-concept? CONCEPT: ~aGRD: ~a" CONCEPT GRD)
  (if (is-member? GRD (get-members CONCEPT))
      (stv 1 1)
      (stv 0 1)))

(define-public (chatlang-choices? CHOICES . GRD)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices."
  (cog-logger-debug chatlang-logger
    "In chatlang-choices? CHOICES: ~aGRD: ~a" CHOICES GRD)
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

(define-public (chatlang-negation? . TERMS)
  "Check if the input sentence has none of the terms specified."
  (let* ; Get the raw text input
        ((sent (car (cog-chase-link 'StateLink 'SentenceNode chatlang-anchor)))
         (rtxt (cog-name (car (cog-chase-link 'ListLink 'Node sent))))
         (ltxt (string-join (map get-lemma (string-split rtxt #\sp)))))
        (if (any (lambda (t) (text-contains? rtxt ltxt t)) TERMS)
            (stv 0 1)
            (stv 1 1))))

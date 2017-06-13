;; ChatLang DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
             (opencog eva-behavior)
             (srfi srfi-1)
             (rnrs io ports)
             (ice-9 popen)
             (ice-9 optargs))

(define-public (chatlang-prefix STR) (string-append "Chatlang: " STR))
(define chatlang-anchor (Anchor (chatlang-prefix "Currently Processing")))
(define chatlang-no-constant (Node (chatlang-prefix "No constant terms")))
(define chatlang-word-seq (Predicate (chatlang-prefix "Word Sequence")))
(define chatlang-lemma-seq (Predicate (chatlang-prefix "Lemma Sequence")))
(define chatlang-grd-words (Anchor (chatlang-prefix "Grounded Words")))
(define chatlang-grd-lemmas (Anchor (chatlang-prefix "Grounded Lemmas")))

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

(define (process-pattern-term TERM ATOMESE)
  "Process a single term -- calls the term function and appends the new
   variables and conditions to the existing pair.
   The atomese are in the form of: ((A B) C)
   where A is the variable declaration, B is the condition, C is the
   term sequence."
  (let* ((no-var-cond (cons '() '()))
         (atomese-for-term
           (cond ((equal? 'lemma (car TERM))
                  (cons (lemma (cdr TERM))
                        ; Use the lemma of a word for the term-seq
                        (list (Word (get-lemma (cdr TERM))))))
                 ((equal? 'word (car TERM))
                  (cons (word (cdr TERM))
                        ; Use the lemma of a word for the term-seq
                        (list (Word (get-lemma (cdr TERM))))))
                 ((equal? 'phrase (car TERM))
                  (cons (phrase (cdr TERM))
                        (map Word (string-split (cdr TERM) #\ ))))
                 ((equal? 'concept (car TERM))
                  (let ((var (choose-var-name)))
                       (cons (concept (cdr TERM) var)
                             (list (Glob var)))))
                 ((equal? 'choices (car TERM))
                  (let ((var (choose-var-name)))
                       (cons (choices (cdr TERM) var)
                             (list (Glob var)))))
                 ((equal? 'unordered-matching (car TERM))
                  (let ((var (choose-var-name)))
                       (cons (unordered-matching (cdr TERM) var)
                             (list (Glob var)))))
                 ((equal? 'negation (car TERM))
                  (cons (negation (cdr TERM)) '()))
                 ((equal? 'anchor-start (car TERM))
                  (cons no-var-cond (list "<")))
                 ((equal? 'anchor-end (car TERM))
                  (cons no-var-cond (list ">")))))
         (vars (append (caar ATOMESE) (caar atomese-for-term)))
         (conds (append (cdar ATOMESE) (cdar atomese-for-term)))
         (seq (append (cdr ATOMESE) (cdr atomese-for-term))))
  (cons (cons vars conds) seq)))

(define (term-sequence-check SEQ)
  "Checks terms occur in the desired order. This is done when we're using
   DualLink to find the rules, see 'find-chat-rules' for details."
  (let* ((start-anchor? (not (equal? #f (member "<" SEQ))))
         (end-anchor? (not (equal? #f (member ">" SEQ))))
         (start-with
           (if start-anchor?
               (cdr (member "<" SEQ))
               (wildcard 0 -1)))
         (end-with
           (if end-anchor?
               (take-while (lambda (t) (not (equal? ">" t))) SEQ)
               (wildcard 0 -1)))
         (mid-wc (if (or start-anchor? end-anchor?) (wildcard 0 -1) '()))
         (glob-decl (append (if start-anchor? '() (car start-with))
                            (if end-anchor? '() (car end-with))
                            (if (null? mid-wc) '() (car mid-wc))))
         (new-seq (cond ; If there are both start-anchor and end-anchor
                        ; they are the whole seq, but still need to put
                        ; a glob in between them
                        ((and start-anchor? end-anchor?)
                         (append start-with (cdr mid-wc) end-with))
                        ; If there is only a start-anchor, append it and
                        ; a wildcard with the main seq, follow by a glob
                        ; at the end
                        (start-anchor?
                         (append start-with
                                 (cdr mid-wc)
                                 (take-while (lambda (t) (not (equal? "<" t))) SEQ)
                                 (cdr end-with)))
                        ; If there is only an end-anchor, append a glob in
                        ; front of the main seq, follow by a wildcard and
                        ; the end-seq
                        (end-anchor?
                         (append (cdr start-with)
                                 (cdr (member ">" SEQ))
                                 (cdr mid-wc)
                                 end-with))
                        ; If there is no anchor, append two globs, one in
                        ; the beginning and one at the end of the seq
                        (else (append (cdr start-with) SEQ (cdr end-with))))))
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that they can be found
  ; easily during the matching process
  (if (equal? (length new-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              new-seq)))
    (Inheritance (List new-seq) chatlang-no-constant))
  (cons glob-decl
        (list (Evaluation chatlang-lemma-seq
                          (List (Variable "$S") (List new-seq)))))))

(define-public (say TXT)
  "Say the text and clear the state."
  ; TODO: Something simplier?
  (And (True (Put (DefinedPredicate "Say") (Node TXT)))
       (True (Put (State chatlang-anchor (Variable "$x"))
                  (Concept "Default State")))))

(define (process-action ACTION)
  "Process a single action -- converting it into atomese."
  (cond ((equal? 'say (car ACTION))
         (say (cdr ACTION)))))

(define-public (store-groundings SENT GRD)
  "Store the groundings, both original words and lemmas,
   for each of the GlobNode in the pattern, by using StateLinks.
   They will be referenced in the stage of evaluating the context
   of the psi-rules, or executing the action of the psi-rules."
  (let ((sent-word-seq (cog-outgoing-set (car (sent-get-word-seqs SENT))))
        (cnt 0))
       (for-each (lambda (g)
         (if (equal? 'ListLink (cog-type g))
             (if (equal? (gar g) (gadr g))
                 ; If the grounded value is the GlobNode itself,
                 ; that means the GlobNode is grounded to nothing
                 (begin
                   (State (Set (gar g) chatlang-grd-words) (List))
                   (State (Set (gar g) chatlang-grd-lemmas) (List)))
                 ; Store the GlobNode and the groundings
                 (begin
                   (State (Set (gar g) chatlang-grd-words)
                          (List (take (drop sent-word-seq cnt)
                                      (length (cog-outgoing-set (gdr g))))))
                   (State (Set (gar g) chatlang-grd-lemmas) (gdr g))
                   (set! cnt (+ cnt (length (cog-outgoing-set (gdr g)))))))
             ; Move on if it's not a GlobNode
             (set! cnt (+ cnt 1))))
         (cog-outgoing-set GRD))))

(define (generate-bind TERM-SEQ)
  "Generate a BindLink that contains the term-seq and the
   restrictions on the GlobNode in the term-seq, if any."
  (Bind (VariableList (car TERM-SEQ)
                      (TypedVariable (Variable "$S")
                                     (Type "SentenceNode")))
        (And (cdr TERM-SEQ)
             (State chatlang-anchor (Variable "$S")))
        (ExecutionOutput (GroundedSchema "scm: store-groundings")
                         (List (Variable "$S")
                               (List (map (lambda (x)
                                 (if (equal? 'GlobNode (cog-type x))
                                     (List (Quote x) (List x))
                                     x))
                                 (cog-outgoing-set (gddr (cadr TERM-SEQ)))))))))

(define* (chat-rule PATTERN ACTION #:optional (TOPIC default-topic) NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((template (cons atomese-variable-template atomese-condition-template))
         (proc-terms (fold process-pattern-term
                           (cons template '())
                           PATTERN))
         (term-seq (term-sequence-check (cdr proc-terms)))
         (var-list (caar proc-terms))
         (cond-list (cdar proc-terms))
         (action (process-action ACTION)))
        ; XXX TODO: term-seq does not have all the glob decl
        (List (generate-bind term-seq)
              (psi-rule-nocheck
                (list (Satisfaction (VariableList var-list) (And cond-list)))
                action
                (True)
                (stv .9 .9)
                TOPIC
                NAME))))

(define (sent-get-word-seqs SENT)
  "Get the words (original and lemma) associate with SENT.
   It also creates an EvaluationLink linking the
   SENT with the word-list and lemma-list."
  (define (get-seq TYPE)
    (List (append-map
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
      (car (sent-get-words-in-order SENT)))))
  (let ((word-seq (get-seq 'ReferenceLink))
        (lemma-seq (get-seq 'LemmaLink)))
       ; These EvaluationLinks will be used in the matching process
       (Evaluation chatlang-word-seq (List SENT word-seq))
       (Evaluation chatlang-lemma-seq (List SENT lemma-seq))
       (cons word-seq lemma-seq)))

(define (get-lemma WORD)
  "A hacky way to quickly find the lemma of a word using WordNet."
  (let* ((cmd-string (string-append "wn " WORD " | grep \"Information available for .\\+\""))
         (port (open-input-pipe cmd-string))
         (lemma ""))
    (do ((line (get-line port) (get-line port)))
        ((eof-object? line))
      (let ((l (car (last-pair (string-split line #\ )))))
        (if (not (equal? (string-downcase WORD) l))
          (set! lemma l))))
    (close-pipe port)
    (if (string-null? lemma) WORD lemma)))

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

(define (is-member? GLOB LST)
  "Check if GLOB is a member of LST, where LST may contain
   WordNodes, LemmaNodes, and PhraseNodes."
  ; TODO: GLOB is grounded to lemmas but not the original
  ; words in the input, this somehow needs to be fixed...
  (let* ((glob-txt-lst (map cog-name GLOB))
         (raw-txt (string-join glob-txt-lst))
         (lemma-txt (string-join (map get-lemma glob-txt-lst))))
    (any (lambda (t)
           (or (and (eq? 'WordNode (cog-type t))
                    (equal? raw-txt (cog-name t)))
               (and (eq? 'LemmaNode (cog-type t))
                    (equal? lemma-txt (cog-name t)))
               (and (eq? 'PhraseNode (cog-type t))
                    (equal? raw-txt (cog-name t)))))
         LST)))

(define-public (chatlang-concept? CONCEPT . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept."
  (cog-logger-debug "In chatlang-concept? GLOB: ~a" GLOB)
  (if (is-member? GLOB (get-members CONCEPT))
      (stv 1 1)
      (stv 0 1)))

(define-public (chatlang-choices? CHOICES . GLOB)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices."
  (cog-logger-debug "In chatlang-choices? GLOB: ~a" GLOB)
  (let* ((chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (is-member? GLOB (append chs cpts))
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

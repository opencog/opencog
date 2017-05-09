;; ChatLang DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
; XXX Testing
;             (opencog eva-behavior)
             (srfi srfi-1)
             (rnrs io ports)
             (ice-9 popen)
             (ice-9 optargs))

(define (chatlang-prefix STR) (string-append "Chatlang: " STR))
(define chatlang-anchor (Anchor (chatlang-prefix "Currently Processing")))
(define chatlang-no-constant (Node (chatlang-prefix "No constant terms")))
(define chatlang-term-seq (Node (chatlang-prefix "term seq")))

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
         ; Use the lemma of a word for the term-seq
         (atomese-for-term
           (cond ((equal? 'lemma (car TERM))
                  (cons (lemma (cdr TERM))
                        (list (Word (get-lemma (cdr TERM))))))
                 ((equal? 'word (car TERM))
                  (cons (word (cdr TERM))
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
                  (cons (unordered-matching (cdr TERM))
                        (list (Glob (choose-var-name)))))
                 ((equal? 'negation (car TERM))
                  (cons (negation (cdr TERM))
                        '()))
                 ((equal? 'anchor-start (car TERM))
                  (cons no-var-cond (list "<")))
                 ((equal? 'anchor-end (car TERM))
                  (cons no-var-cond (list ">")))
                 ; TODO
                 (else (cons no-var-cond '()))))
         (vars (append (caar ATOMESE) (caar atomese-for-term)))
         (conds (append (cdar ATOMESE) (cdar atomese-for-term)))
         (seq (append (cdr ATOMESE) (cdr atomese-for-term))))
  (cons (cons vars conds) seq)))

(define (term-sequence-check SEQ)
  "Checks terms occur in the desired order. This is done when we're using
   DualLink to find the rules, see 'find-chat-rules' for details."
  (let* ((no-start-anchor? (equal? #f (member "<" SEQ)))
         (no-end-anchor? (equal? #f (member ">" SEQ)))
         (start-with (if no-start-anchor? '() (cdr (member "<" SEQ))))
         (end-with (if no-end-anchor?
                       '()
                       (take-while (lambda (t) (not (equal? ">" t))) SEQ)))
         (new-seq (cond ((and (not (null? start-with)) (not (null? end-with)))
                         (append start-with end-with))
                        ; TODO: Append a zero-to-many-GlobNode at the end
                        ((not (null? start-with))
                         (append start-with
                                 (take-while (lambda (t) (not (equal? "<" t))) SEQ)))
                        ; TODO: Append a zero-to-many-GlobNode at the beginning
                        ((not (null? end-with))
                         (append (cdr (member ">" SEQ)) end-with))
                        ; TODO: Append zero-to-many-GlobNodes
                        (else SEQ))))
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that ot cam be found
  ; easily during the matching process
  (if (equal? (length new-seq)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              new-seq)))
    (Inheritance (List new-seq) chatlang-no-constant))
  ; TODO: Wrap it using an TrueLink for now, use something better instead?
  (Inheritance (List new-seq) chatlang-term-seq)
  (True (List new-seq))))

(define-public (say TXT)
  "Say the text and clear the state"
  ; TODO: Something simplier?
;  (And (True (Put (DefinedPredicate "Say") (Node TXT)))
; XXX Testing
(And (Evaluation (GroundedPredicate "scm: sss") (List (Node TXT)))
       (True (Put (State chatlang-anchor (Variable "$x"))
                  (Concept "Default State")))))

; XXX Testing
(define-public (sss TTT)
(display (cog-name TTT)) (newline) (stv 1 1))

(define (process-action ACTION)
  "Process a single action -- converting it into atomese."
  (cond ((equal? 'say (car ACTION))
         (say (cdr ACTION)))))

(define yakking (psi-demand "Yakking" 0.9))

(define*-public (chat-rule PATTERN ACTION #:optional NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((template (cons atomese-variable-template atomese-condition-template))
         (proc-terms (fold process-pattern-term
                           (cons template '())
                           PATTERN))
         (var-list (caar proc-terms))
         (cond-list (cdar proc-terms))
         (term-seq (term-sequence-check (cdr proc-terms)))
         (action (process-action ACTION)))
    (psi-rule
      (list (Satisfaction (VariableList var-list)
                          (And (append cond-list (list term-seq)))))
      action
      (True)
      (stv .9 .9)
      yakking
      NAME)))

(define (sent-get-lemmas-in-order SENT)
  "Get the lemma of the words associate with sent-node."
  (List (append-map
    (lambda (w)
      ; Ignore LEFT-WALL and punctuations
      (if (or (string-prefix? "LEFT-WALL" (cog-name w))
              (word-inst-match-pos? w "punctuation")
              (null? (cog-chase-link 'LemmaLink 'WordNode w)))
          '()
          ; For proper names, e.g. Jessica Henwick,
          ; RelEx converts them into a single WordNode, e.g.
          ; (WordNode "Jessica_Henwick"). Codes below try to
          ; split it into two WordNodes, "Jessica" and "Henwick",
          ; so that the matcher will be able to find the rules
          (let* ((wn (car (cog-chase-link 'LemmaLink 'WordNode w)))
                 (name (cog-name wn)))
            (if (integer? (string-index name #\_))
              (map Word (string-split name  #\_))
              (list wn)))))
    (car (sent-get-words-in-order SENT)))))

(define (get-lemma WORD)
  "A hacky way to quickly find the lemma of a word using WordNet."
  (let* ((cmd-string (string-append "wn " WORD " | grep \"Information available for .\\+\""))
         (port (open-input-pipe cmd-string))
         (lemma ""))
    (do ((line (get-line port) (get-line port)))
        ((eof-object? line))
      (let ((l (car (last-pair (string-split line #\ )))))
        (if (not (equal? WORD l))
          (set! lemma l))))
    (close-pipe port)
    (if (string-null? lemma) WORD lemma)))

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

(define-public (chatlang-concept? GLOB CONCEPT)
  "Check if the value grounded for the GlobNode is actually a member
   of the concept."
  (let ((grd (assoc-ref globs (cog-name GLOB)))
        (membs (get-members CONCEPT)))
       (if (not (equal? #f (member grd membs)))
           (stv 1 1)
           (stv 0 1))))

(define-public (chatlang-choices? GLOB CHOICES)
  "Check if the value grounded for the GlobNode is actually a member
   of the list of choices."
  (let* ((grd (assoc-ref globs (cog-name GLOB)))
         (chs (cog-outgoing-set CHOICES))
         (cpts (append-map get-members (cog-filter 'ConceptNode chs))))
        (if (not (equal? #f (member grd (append chs cpts))))
            (stv 1 1)
            (stv 0 1))))

(define (text-contains? TXT TERM)
  "Check if TXT contains TERM."
  (define (contains? txt term)
    (not (equal? #f (regexp-exec
      (make-regexp (string-append "\\b" term "\\b") regexp/icase) txt))))
  (cond ((equal? 'WordNode (cog-type TERM))
         (contains? TXT (cog-name TERM)))
        ((equal? 'ListLink (cog-type TERM))
         (contains? TXT (string-join (map cog-name (cog-outgoing-set TERM)) " ")))
        ((equal? 'ConceptNode (cog-type TERM))
         (any (lambda (t) (text-contains? TXT t))
              (get-members TERM)))))

(define-public (chatlang-negation? . TERMS)
  "Check if the input sentence has none of the terms specified."
  (let* ; Get the raw text input
        ((sent (car (cog-chase-link 'StateLink 'SentenceNode chatlang-anchor)))
         (itxt (cog-name (car (cog-chase-link 'ListLink 'Node sent)))))
        (if (any (lambda (t) (text-contains? itxt t)) TERMS)
            (stv 0 1)
            (stv 1 1))))

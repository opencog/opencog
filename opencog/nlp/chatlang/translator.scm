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

(define (chatlang-prefix str) (string-append "Chatlang: " str))
(define chatlang-no-constant (Node (chatlang-prefix "No constant terms")))

;; Shared variables for all terms
(define atomese-variable-template (list (TypedVariable (Variable "$S")
                                                       (Type "SentenceNode"))
                                        (TypedVariable (Variable "$P")
                                                       (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template (list (Parse (Variable "$P")
                                                (Variable "$S"))
                                         (State (Anchor "Currently Processing")
                                                (Variable "$S"))))

(define (process-pattern-term term atomese-pattern)
  "Process a single term -- calls the term function and appends the new
   variables and conditions to the existing pair."
  (let* ((atomese-for-term (primitive-eval term))
         (vars (append (car atomese-pattern) (car atomese-for-term)))
         (conds (append (cdr atomese-pattern) (cdr atomese-for-term))))
    (cons vars conds)))

(define (term-sequence-check terms)
  "Checks terms occur in the desired order. This is done when we're using
   DualLink to find the rules, see 'find-chat-rules' for details."
  ; A hacky way to quickly find the lemma of a word using WordNet...
  (define (get-lemma word)
    (let* ((cmd-string (string-append "wn " word " | grep \"Information available for .\\+\""))
           (port (open-input-pipe cmd-string))
           (lemma ""))
      (do ((line (get-line port) (get-line port)))
          ((eof-object? line))
        (let ((l (car (last-pair (string-split line #\ )))))
          (if (not (equal? word l))
            (set! lemma l))))
      (close-pipe port)
      (if (string-null? lemma) word lemma)))
  (define word-list
    (append-map (lambda (w)
      (cond ((equal? 'concept (car w)) (list (Glob (cadr w))))
            ; Skip the sentence anchors, they will be handled later
            ((equal? 'anchor-start (car w)) '())
            ((equal? 'anchor-end (car w)) '())
            ; For proper names -- create WordNodes
            ((equal? 'proper-names (car w)) (map Word (cdr w)))
            ((equal? 'or-choices (car w)) (list (Glob "$choices")))
            ((equal? 'unordered-matching (car w)) (list (Glob "$unordered")))
            ((not (equal? #f (string-index (cadr w) char-upper-case?)))
             (list (Word (cadr w))))
            (else (list (Word (get-lemma (cadr w)))))))
         terms))
  ; Append the words in 'start-with' and 'end-with' to word-list, if any
  (set! word-list (append start-with word-list end-with))
  ; DualLink couldn't match patterns with no constant terms in it
  ; Mark the rules with no constant terms so that ot cam be found
  ; easily during the matching process
  (if (equal? (length word-list)
              (length (filter (lambda (x) (equal? 'GlobNode (cog-type x)))
                              word-list)))
    (Inheritance (List word-list) chatlang-no-constant))
  ; Wrap it using a TrueLink
  ; TODO: Maybe there is a more elegant way to represent it in the context?
  (True (List word-list)))

(define-public (say text)
  "Say the text and clear the state"
  (And (True (Put (DefinedPredicate "Say") (Node text)))
       (True (Put (State (Anchor "Currently Processing") (Variable "$x"))
                  (Concept "Default State")))))

(define yakking (psi-demand "Yakking" 0.9))

(define*-public (chat-rule pattern action #:optional name)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((template (cons atomese-variable-template atomese-condition-template))
         (proc-terms (fold process-pattern-term
                           template
                           pattern))
         ; There may be duplicates if the pattern contains any two or more
         ; of the main-* terms, e.g. main-verb, main-subj, and main-obj
         (var-list (delete-duplicates (car proc-terms)))
         (cond-list (delete-duplicates (cdr proc-terms)))
         (seq-check (term-sequence-check pattern)))
    (psi-rule
      (list (Satisfaction (VariableList var-list)
                          (And (append cond-list (list seq-check)))))
      (primitive-eval action)
      (True)
      (stv .9 .9)
      yakking
      name)))

(define (member-words w)
  (let ((words (string-split w #\sp)))
    (if (= 1 (length words))
        (Word (car words))
        (List (map-in-order Word words)))))

(define-public (chat-concept name members)
  "Lets users create named concepts with explicit membership lists."
  (let* ((c (Concept name))
         (ref-members (append-map (lambda (m) (list (Reference (member-words m) c)))
                                  members)))
    ref-members))

(define (get-sent-lemmas sent-node)
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
    (car (sent-get-words-in-order sent-node)))))

(define-public (does-not-contain sent list-of-words)
  "Check if the given sentence contains any of the listed words.
   Return true if it contains none."
  (let ((sent-text (cog-name (car (cog-chase-link 'ListLink 'Node sent)))))
    (if (null? (filter
      (lambda (w) (not (equal? #f (regexp-exec (make-regexp
        (string-append "\\b" (cog-name w) "\\b") regexp/icase) sent-text))))
          (cog-outgoing-set list-of-words)))
      (stv 1 1)
      (stv 0 1))))

(define-public (does-not-start-with sent list-of-words)
  "Check if the given sentence starts with any of the listed words.
   Return true if it starts with none of the words."
  (let ((sent-text (cog-name (car (cog-chase-link 'ListLink 'Node sent)))))
    (if (null? (filter
      (lambda (w) (not (equal? #f (regexp-exec (make-regexp
        (string-append "^" (cog-name w) "\\b") regexp/icase) sent-text))))
          (cog-outgoing-set list-of-words)))
      (stv 1 1)
      (stv 0 1))))

(define-public (does-not-end-with sent list-of-words)
  "Check if the given sentence ends with any of the listed words.
   Return true if it ends with none of the words."
  (let ((sent-text (cog-name (car (cog-chase-link 'ListLink 'Node sent)))))
    (if (null? (filter
      (lambda (w) (not (equal? #f (regexp-exec (make-regexp
        (string-append "\\b" (cog-name w) "$") regexp/icase) sent-text))))
          (cog-outgoing-set list-of-words)))
      (stv 1 1)
      (stv 0 1))))

(define-public (no-words-in-between sent w1 w2 list-of-words)
  "Check if the given sentence contains any of the listed words
   between words w1 and w2.
   Return true if it contains none."
  (let ((sent-text (cog-name (car (cog-chase-link 'ListLink 'Node sent))))
        (w1-name (cog-name w1))
        (w2-name (cog-name w2)))
    (if (null? (filter
      (lambda (w) (not (equal? #f (regexp-exec (make-regexp
        (string-append "\\b" w1-name " " (cog-name w) " " w2-name "\\b")
          regexp/icase) sent-text))))
            (cog-outgoing-set list-of-words)))
      (stv 1 1)
      (stv 0 1))))

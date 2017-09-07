(define (gen-var STR LEMMA?)
  "Helper function for generating the name of a VariableNode,
   so as to make it slightly easier for a human to debug the code.
   is-literal? flag is for indicating whether this variable
   or glob is supposed to go to the word-seq or lemma-seq."
  (if LEMMA?
      (string-append (get-lemma STR) "-" (choose-var-name))
      (string-append STR "-" (choose-var-name))))

(define (term-length TERMS)
  "Helper function to find the maximum number of words one of
   the TERMS has, for setting the upper bound of the GlobNode."
  (fold
    (lambda (term len)
      (let ((tl (cond ((equal? 'phrase (car term))
                       (length (string-split (cdr term) #\sp)))
                      ((equal? 'concept (car term))
                       (concept-length (Concept (cdr term))))
                      (else 1))))
           (max len tl)))
    0
    TERMS))

(define (concept-length CONCEPT)
  "Helper function to find the maximum number of words CONCEPT has."
  (define c (cog-outgoing-set (cog-execute!
              (Get (Reference (Variable "$x") CONCEPT)))))
  (if (null? c)
      -1  ; This may happen if the concept is not yet defined in the system...
      (fold (lambda (term len)
        (let ((tl (cond ((equal? 'PhraseNode (cog-type term))
                         (length (string-split (cog-name term) #\sp)))
                        ((equal? 'ConceptNode (cog-type term))
                         (concept-length term))
                        (else 1))))
             (max tl len)))
        0
        c)))

(define (terms-to-atomese TERMS)
  "Helper function to convert a list of terms into atomese.
   For use of choices and negation."
  (map (lambda (t)
    (cond ((equal? 'word (car t))
           (WordNode (cdr t)))
          ((equal? 'lemma (car t))
           (LemmaNode (get-lemma (cdr t))))
          ((equal? 'phrase (car t))
           (PhraseNode (cdr t)))
          ((equal? 'concept (car t))
           (Concept (cdr t)))
          (else (feature-not-supported (car t) (cdr t)))))
       TERMS))

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

(define (text-contains? RTXT LTXT TERM)
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

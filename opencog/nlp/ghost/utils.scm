(define (gen-var STR LEMMA?)
  "Helper function for generating the name of a VariableNode,
   so as to make it slightly easier for a human to debug the code.
   Lemma? flag indicates whether this word should be represented
   in lemma or not."
  (if LEMMA?
      (string-append (get-lemma STR) "-" (choose-var-name))
      (string-append STR "-" (choose-var-name))))

(define (term-length TERMS)
  "Helper function to find the maximum number of words one of
   the TERMS has, for setting the upper bound of the GlobNode."
  (define (get-length term)
    (cond ((equal? 'phrase (car term))
           (length (string-split (cdr term) #\sp)))
          ((equal? 'concept (car term))
           (concept-length (Concept (cdr term))))
          ((equal? 'sequence (car term))
           (apply + (map get-length (cdr term))))
          (else 1)))
  (fold
    (lambda (term len) (max len (get-length term)))
    0
    TERMS))

(define (concept-length CONCEPT)
  "Helper function to find the maximum number of words CONCEPT has."
  (define c (cog-outgoing-set (cog-execute!
              (Get (Reference (Variable "$x") CONCEPT)))))
  (if (null? c)
      -1  ; This may happen if the concept is not yet defined in the system...
      (fold (lambda (term len)
        (let ((tl (cond ((equal? 'ListLink (cog-type term))
                         (length (cog-outgoing-set term)))
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
           (ListLink (map Word (string-split (cdr t) #\sp))))
          ((equal? 'concept (car t))
           (ConceptNode (cdr t)))
          ((equal? 'sequence (car t))
           (List (flatten-list (terms-to-atomese (cdr t)))))
          (else (begin
            (cog-logger-warn ghost-logger
              "Feature not supported: \"(~a ~a)\"" (car t) (cdr t))
            (throw 'FeatureNotSupported (car t) (cdr t))))))
       TERMS))

(define (generate-word-seqs SENT)
  "Get the words and their corresponding lemmas associate with SENT
   and put them into two lists -- word-seq and lemma-seq."
  (define (get-seq TYPE)
    (append-map
      (lambda (w)
        ; Ignore LEFT-WALL and punctuations
        (if (or (string-prefix? "LEFT-WALL" (cog-name w))
                (word-inst-match-pos? w "punctuation")
                (null? (cog-chase-link TYPE 'WordNode w)))
            '()
            (cog-chase-link TYPE 'WordNode w)))
      (car (sent-get-words-in-order SENT))))
  (let* ((wseq (get-seq 'ReferenceLink))
         (word-seq (List wseq))
         (lseq (get-seq 'LemmaLink))
         (lemma-seq (List lseq)))
        ; These EvaluationLinks will be used in the matching process
        (Evaluation ghost-word-seq (List SENT word-seq))
        (Evaluation ghost-lemma-seq (List SENT lemma-seq))))

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

(define (is-member? GRD MEMB)
  "Check if GRD (the grounding of a glob) is a member of MEMB,
   where MEMB may be a WordNode, LemmaNode, or a mix of them."
  (any (lambda (m)
         (or (and (equal? 'WordNode (cog-type m))
                  (equal? 1 (length GRD))
                  (equal? (cog-name m) (cog-name (car GRD))))
             (and (equal? 'LemmaNode (cog-type m))
                  (equal? 1 (length GRD))
                  (equal? (cog-name m) (get-lemma (cog-name (car GRD)))))
             (and (equal? 'ListLink (cog-type m))
                  (equal? (length (cog-outgoing-set m)) (length GRD))
                  (every (lambda (x y) (is-member? (list x) (list y)))
                         GRD (cog-outgoing-set m)))))
       MEMB))

(define (text-contains? RTXT LTXT TERM)
  "Check if either RTXT or LTXT contains the string (name of) TERM."
  (define (contains? txt term)
    (not (equal? #f (regexp-exec
      (make-regexp (string-append "\\b" term "\\b") regexp/icase) txt))))
  (cond ((equal? 'WordNode (cog-type TERM))
         (contains? RTXT (cog-name TERM)))
        ((equal? 'LemmaNode (cog-type TERM))
         (contains? LTXT (cog-name TERM)))
        ((equal? 'ListLink (cog-type TERM))
         (contains? RTXT (string-join (map cog-name (cog-outgoing-set TERM)))))
        ((equal? 'ConceptNode (cog-type TERM))
         (any (lambda (t) (text-contains? RTXT LTXT t))
              (get-members TERM)))))

(define (flatten-list LST)
  "Remove unnecessary ListLink in LST, if any.
   For example, turning:
     (ListLink (Word \"hi\") (ListLink (Word \"you\") (Word \"robot\")))
   into:
     (ListLink (Word \"hi\") (Word \"you\") (Word \"robot\"))"
  (map (lambda (x)
         (if (equal? 'ListLink (cog-type x))
             (cog-outgoing-set x)
             x))
       LST))

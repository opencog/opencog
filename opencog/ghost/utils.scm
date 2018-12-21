;; Helper functions

(define (gen-var STR LEMMA?)
"
  Helper function for generating the name of a VariableNode,
  so as to make it slightly easier for a human to debug the code.
  The LEMMA? flag indicates whether this word should be represented
  in lemma or not.
"
  (if LEMMA?
      (string-append (get-lemma STR) "-" (choose-var-name))
      (string-append STR "-" (choose-var-name))))

; ----------
(define (term-length TERMS)
"
  Helper function to find the maximum number of words one of
  the TERMS has, for setting the upper bound of the GlobNode.
"
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

; ----------
(define (concept-length CONCEPT)
"
  Helper function to find the maximum number of words CONCEPT has.
"
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

; ----------
(define (terms-to-atomese TERMS)
"
  Helper function to convert a list of terms into atomese.
  For use of choices, negation, and topic etc.
"
  (map (lambda (t)
    (cond ((or (equal? 'word (car t)) (equal? 'word-apos (car t)))
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

; ----------
(define (generate-word-seqs SENT)
"
  Get the words and their corresponding lemmas associate with SENT
  and put them into two lists -- word-seq and lemma-seq.
"
  (define input-word-seq (car (sent-get-words-in-order SENT)))

  (define (get-seq TYPE)
    (append-map
      (lambda (w)
        ; Ignore LEFT-WALL and punctuations
        (if (or (string-prefix? "LEFT-WALL" (cog-name w))
                (word-inst-match-pos? w "punctuation")
                (null? (cog-chase-link TYPE 'WordNode w)))
            '()
            (cog-chase-link TYPE 'WordNode w)))
      input-word-seq))

  (define word-seq (get-seq 'ReferenceLink))

  (define final-word-seq '())
  (define word-apos-alist '())

  ; Contraction may be splitted into two WordNodes, because of their linguistic role
  ; e.g. "I'm" -> "I" and "'m"
  ; Merge them back to one, just for matching
  (do ((i 0 (1+ i)))
      ((>= i (length word-seq)))
    (let* ((current-word-node (list-ref word-seq i))
           (current-word-str (cog-name current-word-node))
           (next-word-str
             (if (< (1+ i) (length word-seq))
               (cog-name (list-ref word-seq (1+ i))) ""))
           (current-word-splitted
             (string-split current-word-str (char-set #\' #\’)))
           (next-word-prefix-with-apos?
             (or (string-prefix? "'" next-word-str)
                 (string-prefix? "’" next-word-str))))
      (cond
        (next-word-prefix-with-apos?
         (let ((merged-word (WordNode
                 (string-append
                   current-word-str
                   "'"  ; This will turn "’" into "'", for consistency
                   (string-drop next-word-str 1)))))
           (set! final-word-seq (append final-word-seq (list merged-word)))
           (set! word-apos-alist (assoc-set! word-apos-alist (cons i (1+ i)) merged-word))
           (set! i (1+ i))))
        ; To merge for example "dr" and "." into one word, just for matching
        ((and (string=? next-word-str ".")
              (is-nonbreaking-prefix? current-word-str))
         (set! final-word-seq (append final-word-seq
           (list (WordNode (string-append current-word-str ".")))))
         (set! i (1+ i)))
        ; The current word may also have an apostrophe, make sure to turn
        ; "’" into "'" as well for consistency
        ((> (length current-word-splitted) 1)
         (let ((new-word (WordNode (string-join current-word-splitted "'"))))
           (set! final-word-seq (append final-word-seq (list new-word)))
           (set! word-apos-alist (assoc-set! word-apos-alist (cons i i) new-word))))
        (else (set! final-word-seq (append final-word-seq (list current-word-node)))))))
  (Evaluation
    ghost-word-seq
    (List SENT
      (List (map
        (lambda (w)
          (WordNode (string-downcase (cog-name w))))
        final-word-seq))))

  ; Generate this lemma-seq only for backward compatibility
  (if (not ghost-with-ecan)
    (let (; In some rare situation, particularly if the input
          ; sentence is not grammatical, RelEx may not lemmatize a
          ; word because of the ambiguity.
          ; As a result the lemma sequence may contain non-lemmatized
          ; words, which will become a problem during rule-matching.
          ; As a quick workaround, do "ghost-get-lemma" for each of
          ; the words in the lemma sequence
          (lemma-seq (cog-outgoing-set
            (apply ghost-get-lemma
              ; For idioms, they will be joined by a "_",
              ; e.g. "allows_for"
              ; Split it so that DualLink can find the rule
              (append-map
                (lambda (w)
                  (if (equal? #f (string-contains (cog-name w) "_"))
                    (list w)
                    (map Word (string-split (cog-name w) #\_))))
                (get-seq 'LemmaLink)))))
          (final-lemma-seq '()))
      ; Get the contractions found above, if any, and put it in the lemma-seq
      ; Use their original form in matching, instead of their lemmas
      (do ((i 0 (1+ i)))
          ((>= i (length lemma-seq)))
        (set! final-lemma-seq (append final-lemma-seq (list
          (cond
            ; For the next-word-prefix-with-apos? case
            ((assoc-ref word-apos-alist (cons i (1+ i)))
             (set! i (1+ i))
             (assoc-ref word-apos-alist (cons (1- i) i)))
            ; For having apos in the same word
            ((assoc-ref word-apos-alist (cons i i))
             (assoc-ref word-apos-alist (cons i i)))
            ; For nonbreaking-prefix like Mr. Mrs. etc
            ((and (< (1+ i) (length lemma-seq))
                  (string=? "." (cog-name (list-ref lemma-seq (1+ i))))
                  (is-nonbreaking-prefix? (cog-name (list-ref lemma-seq i))))
             (set! i (1+ i))
             (WordNode (string-append (cog-name (list-ref lemma-seq (1- i))) ".")))
            ; For time, regardless of the format e.g. "2am", "2 am", "2 a.m." or "2a.m.",
            ; will all get splitted into two words, and this is a quick workaround for
            ; the problem of RelEx lemmatizing "a.m." to "be"
            ((and (< (1+ i) (length lemma-seq))
                  (string->number (cog-name (list-ref lemma-seq i)))
                  (string=? "be" (cog-name (list-ref lemma-seq (1+ i)))))
             (set! i (1+ i))
             (list (list-ref lemma-seq (1- i)) (list-ref final-word-seq i)))
            ; Just a normal word
            (else (list-ref lemma-seq i)))))))
    (Evaluation
      ghost-lemma-seq
      (List SENT
        (List (map
          (lambda (w)
            (WordNode (string-downcase (cog-name w))))
          (flatten final-lemma-seq))))))))

; ----------
(define (get-lemma-from-relex WORD)
"
  Get the lemma of WORD via the RelEx server.
"
  (relex-parse WORD)
  (let* ((sent (car (get-new-parsed-sentences)))
         (word-inst (cadar (sent-get-words-in-order sent)))
         (lemma (cog-name (car (cog-chase-link 'LemmaLink 'WordNode word-inst)))))
    (release-new-parsed-sents)
    (if (equal? (string-downcase WORD) lemma)
        WORD
        lemma)))

; ----------
(define (get-lemma WORD)
"
  Get the lemma of WORD and store it in a cache.
  If the same word has been seen before, it will return the one in cache so
  the query won't be sent to the RelEx server.
"
  (define seen-lemma (assoc-ref lemma-alist WORD))
  (if (equal? #f seen-lemma)
      (let ((lemma
              ; Don't bother if it's, say, a personal title like "Mrs."
              ; or it's time related like a.m. and p.m.
              (if (or (is-nonbreaking-prefix? WORD)
                      (string=? "a.m." WORD)
                      (string=? "p.m." WORD))
                WORD
                (get-lemma-from-relex WORD))))
        (set! lemma-alist (assoc-set! lemma-alist WORD lemma))
        lemma)
      seen-lemma))

; ----------
(define (is-lemma? WORD)
"
  Check if WORD is a lemma.
"
  (equal? WORD (get-lemma WORD)))

; ----------
(define (get-members CONCEPT)
"
  Get the members of a concept. VariableNodes will be ignored, and
  recursive calls will be made in case there are nested concepts.
"
  (append-map
    (lambda (g)
      (cond ((eq? 'ConceptNode (cog-type g)) (get-members g))
            ((eq? 'VariableNode (cog-type g)) '())
            (else (list g))))
    (cog-outgoing-set
      (cog-execute! (Get (Reference (Variable "$x") CONCEPT))))))

; ----------
(define (is-member? GRD MEMB)
"
  Check if GRD (the grounding of a glob) is a member of MEMB,
  where MEMB may be a WordNode, LemmaNode, or a mix of them.
"
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

; ----------
(define (text-contains? RTXT LTXT TERM)
"
  Check if either RTXT or LTXT contains the string (name of) TERM.
"
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

; ----------
(define (flatten-list LST)
"
  Remove unnecessary ListLink in LST, if any.
  For example, turning:
    (ListLink (Word \"hi\") (ListLink (Word \"you\") (Word \"robot\")))
  into:
    (ListLink (Word \"hi\") (Word \"you\") (Word \"robot\"))
"
  (append-map
    (lambda (x)
      (if (equal? 'ListLink (cog-type x))
        (flatten-list (cog-outgoing-set x))
        (list x)))
    LST))

; ----------
(define (flatten LST)
"
  Flatten a list of lists.
"
  (cond ((null? LST) '())
        ((pair? (car LST))
         (append (flatten (car LST))
                 (flatten (cdr LST))))
        (else (cons (car LST) (flatten (cdr LST)))))
)

; ----------
(define (get-rejoinder-level TYPE)
"
  Return the rejoinder level, e.g. a = level 1, b = level 2, and so on...
"
  (- (char->integer TYPE) 96))

; ----------
(define (get-rule-topic RULE)
"
  Get which topic(s) the RULE is in.
"
  (filter
    (lambda (x)
      (any (lambda (y) (equal? ghost-topic y))
           (cog-chase-link 'InheritanceLink 'ConceptNode x)))
    (cog-chase-link 'InheritanceLink 'ConceptNode RULE)))

; ----------
(define (is-rule-in-topic? RULE TOPIC)
"
  Check if RULE is a member of TOPIC.

  It is not impossible that the exact same rule exists in
  multiple topics, so get and check all of them.
"
  (any (lambda (t) (equal? TOPIC t)) (get-rule-topic RULE)))

; ----------
(define (topic-has-feature? TOPIC FEATURE)
"
  Check if TOPIC has a certain feature named FEATURE.
"
  (not (equal? #f
    (find (lambda (f) (equal? (StringValue FEATURE) f))
          (cog-value->list (cog-value TOPIC ghost-topic-feature))))))

; ----------
(define (get-related-psi-rules ATOM)
"
  Get all the psi-rules that contain ATOM
"
  (filter psi-rule? (cog-get-trunk ATOM)))

; ----------
(define (get-rule-from-label LABEL)
"
  Given the label of a rule in string, return the rule with that label.
"
  (define rule (filter psi-rule?
    (cog-chase-link 'ListLink 'ImplicationLink
      (Concept LABEL))))

  (if (null? rule)
      (begin
        (cog-logger-debug ghost-logger
          "Failed to find the GHOST rule \"~a\"" LABEL)
        (list))
      (car rule)))

; ----------
(define (is-nonbreaking-prefix? WORD)
"
  Check if WORD is a personal title.
"
  (define lst (list
    "abp" "adj" "adm" "adv" "asst" "bart" "bp" "bldg" "brig" "bros"
    "capt" "cmdr" "col" "comdr" "con" "corp" "cpl" "dr" "drs" "ens"
    "gen" "gov" "hon" "hr" "hosp" "insp" "lt" "maj" "messrs" "mlle"
    "mm" "mme" "mr" "mrs" "ms" "msgr" "op" "ord" "pfc" "ph" "prof"
    "pvt" "rep" "reps" "res" "rev" "rt" "sen" "sens" "sfc" "sgt"
    "sr" "st" "supt" "surg"
    "abstr" "acad" "acct" "accts" "admin" "agric" "amer" "ar" "arch"
    "assn" "assoc" "cong" "dept" "econ" "ed" "ess" "evang" "fr"
    "gaz" "glac" "gr" "hist" "hosp" "inst" "jas" "let" "lett" "libr"
    "mss" "mt" "org" "phys" "princ" "proc" "prod" "prol" "prov" "pt"
    "publ" "quot" "quots" "ref" "reg" "rep" "rept" "rev" "roy" "russ"
    "soc" "tel" "ths" "trad" "transl" "univ" "will" "wk" "wkly" "wlky"
    "wks" "wm" "yr" "zool" "v" "vs" "i.e" "e.g" "op" "cit" "p.s" "q.v"
    "viz" "no" "nos" "art" "nr" "pp" "fig" "i" "ii" "p" "seq" "sp"
    "spec" "specif" "vol" "vols"))

  (or (member (string-downcase WORD) lst)
      (and (string-suffix? "." WORD)
           (member (string-downcase (car (string-split WORD #\.))) lst)))
)

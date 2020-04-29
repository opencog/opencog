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
              (Get (Member (Variable "$x") CONCEPT)))))

  (if (nil? c)
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
  For use of choices, negation etc.
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
          ((equal? 'get_uvar (car t))
           (get-user-variable (cdr t)))
          ((equal? 'sequence (car t))
           (List (flatten-list (terms-to-atomese (cdr t)))))
          (else (begin
            (cog-logger-warn ghost-logger
              "Feature not supported: \"(~a ~a)\"" (car t) (cdr t))
            (throw 'FeatureNotSupported (car t) (cdr t))))))
       TERMS))

; ----------
(define (generate-word-seq SENT)
"
  Get the words and generate a word-seq for matching.
"
  (define input-word-seq (car (sent-get-words-in-order SENT)))

  (define (get-seq TYPE)
    (append-map
      (lambda (w)
        ; Ignore LEFT-WALL and punctuations
        (if (or (string-prefix? "LEFT-WALL" (cog-name w))
                (word-inst-match-pos? w "punctuation")
                (nil? (cog-chase-link TYPE 'WordNode w)))
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
          (define word-str (cog-name w))
          (define wordnode-dc (WordNode (string-downcase word-str)))
          (cog-set-value!
            wordnode-dc
            ghost-word-original
            (WordNode word-str))
          wordnode-dc)
        final-word-seq)))))

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
      (cog-execute! (Get (Member (Variable "$x") CONCEPT))))))

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
                  (equal? (length (flatten-list (cog-outgoing-set m))) (length GRD))
                  (every (lambda (x y) (is-member? (list x) (list y)))
                         GRD (flatten-list (cog-outgoing-set m))))))
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
  (cond ((nil? LST) '())
        ((pair? (car LST))
         (append (flatten (car LST))
                 (flatten (cdr LST))))
        (else (cons (car LST) (flatten (cdr LST)))))
)

; ----------
(define (flatten-linkval LV)
"
  Given a LinkValue LV, flatten any nested LinkValues,
  with an assumption that the values are all atoms.

  A Scheme list will be returned as a result.
"
  (append-map
    (lambda (x)
      (if (cog-atom? x)
        (list x)
        (flatten-linkval x)))
    (cog-value->list LV)))

; ----------
(define (get-rejoinder-level TYPE)
"
  Return the rejoinder level, e.g. j1 = level 1, j2 = level 2, and so on...
"
  (if (string-prefix? "j" TYPE)
    (string->number (string-trim TYPE (lambda (c) (eqv? c #\j))))
    ; For backward compatibility,
    ; e.g. a = level 1, b = level 2, and so on...
    (- (char->integer (string-ref TYPE 0)) 96)))

; ----------
(define (get-related-psi-rules ATOM)
"
  Get all the psi-rules that contain ATOM
"
  (filter psi-rule? (cog-get-trunk ATOM)))

; ----------
(define (get-rules-from-label LABEL)
"
  Given the label of a rule in string, return the psi-rule(s) with that label.
"
  (define rules
    (map gar (filter
      (lambda (x)
        (and (psi-rule? (gar x))
             (any (lambda (p) (string=? "alias" (cog-name p)))
               (cog-chase-link 'EvaluationLink 'PredicateNode x))))
      (cog-incoming-by-type (Concept LABEL) 'ListLink))))

  (if (nil? rules)
      (begin
        (cog-logger-debug ghost-logger
          "Failed to find any GHOST rule with label \"~a\"" LABEL)
        (list))
      rules))

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

; ----------
(define (current-time-us)
"
  Returns the current-time in microseconds.
"
  (define t (gettimeofday))
  (+ (car t) (/ (cdr t) 1000000))
)

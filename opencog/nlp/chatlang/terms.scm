;; ChatLang DSL for chat authoring rules
;;
;; Assorted functions for translating individual terms into Atomese fragments.

(use-modules (ice-9 optargs))

(define (word STR)
  "Literal word occurrence."
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         (c (list (ReferenceLink (Variable name) (WordNode STR))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define (lemma STR)
  "Lemma occurrence, aka canonical form of a term. This is the default
   for word mentions in the rule pattern."
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         ; Note: This converts STR to its lemma
         (c (list (LemmaLink (Variable name) (WordNode (get-lemma STR)))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define (phrase STR)
  "Occurrence of a phrase or a group of words.
   All the words are assumed to be literal / non-canonical."
  (fold (lambda (wd lst)
                (cons (append (car lst) (car wd))
                      (append (cdr lst) (cdr wd))))
        (cons '() '())
        (map word (string-split STR #\ ))))

(define (get-concept-length CONCEPT)
  "Helper function to find the maximum number of words CONCEPT has."
  (define c (cog-outgoing-set (cog-execute!
              (Get (Reference (Variable "$x") CONCEPT)))))
  (if (null? c)
      0
      (fold (lambda (term len)
        (let ((tl (cond ((equal? 'PhraseNode (cog-type term))
                         (length (string-split (cog-name term) #\sp)))
                        ((equal? 'ConceptNode (cog-type term))
                         (get-concept-length term))
                        (else 1))))
             (max tl len)))
        0
        c)))

(define* (concept STR #:optional (VAR (choose-var-name)))
  "Occurrence of a concept."
  (cons (list (TypedVariable (Glob VAR)
                (TypeSet (Type "WordNode")
                         (Interval (Number 1)
                                   (Number (get-concept-length (Concept STR)))))))
        (list (Evaluation (GroundedPredicate "scm: chatlang-concept?")
                          (List (Concept STR)
                                (Glob VAR))))))

(define (terms-to-atomese TERMS)
  "Helper function to convert a list of terms into atomese.
   For use of choices and negation."
  (map (lambda (t) (cond ((equal? 'word (car t))
                          (WordNode (cdr t)))
                         ((equal? 'lemma (car t))
                          (LemmaNode (get-lemma (cdr t))))
                         ((equal? 'phrase (car t))
                          (PhraseNode (cdr t)))
                         ((equal? 'concept (car t))
                          (Concept (cdr t)))))
       TERMS))

(define (get-term-length TERMS)
  "Helper function to find the maximum number of words one of
   the TERMS has."
  (fold
    (lambda (term len)
      (let ((tl (cond ((equal? 'phrase (car term))
                       (length (string-split (cdr term) #\sp)))
                      ((equal? 'concept (car term))
                       (get-concept-length (Concept (cdr term))))
                      (else 1))))
           (max len tl)))
    0
    TERMS))

(define* (choices TERMS #:optional (VAR (choose-var-name)))
  "Occurrence of a list of choices. Existence of either one of
   the words/lemmas/phrases/concepts in the list will be considered
   as a match."
  (cons (list (TypedVariable (Glob VAR)
                             (TypeSet (Type "WordNode")
                                      (Interval (Number 1)
                                                (Number (get-term-length TERMS))))))
        (list (Evaluation (GroundedPredicate "scm: chatlang-choices?")
                          (List (List (terms-to-atomese TERMS))
                                (Glob VAR))))))

(define* (unordered-matching TERMS #:optional (VAR (choose-var-name)))
  "Occurrence of a list of terms (words/lemmas/phrases/concepts)
   that can be matched in any orders."
  (fold (lambda (t lst)
                (cons (append (car lst) (car t))
                      (append (cdr lst) (cdr t))))
        (cons (list (TypedVariable (Glob VAR)
                        (TypeSet (Type "WordNode")
                                 (Interval (Number (length TERMS))
                                           (Number (length TERMS))))))
              '())
        (map (lambda (t)
          (cond ((equal? 'word (car t))
                 (word (cdr t)))
                ((equal? 'lemma (car t))
                 (lemma (cdr t)))
                ((equal? 'phrase (car t))
                 (phrase (cdr t)))
                ((equal? 'concept (car t))
                 (concept (cdr t)))))
          TERMS)))

(define (negation TERMS)
  "Absent of a term or a list of terms (words/phrases/concepts)."
  (cons '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: chatlang-negation?")
                          (List (terms-to-atomese TERMS))))))

(define* (wildcard LOWER UPPER #:optional (VAR (choose-var-name)))
  "Occurrence of a wildcard that the number of atoms to be matched
   can be restricted. -1 in the upper bound means infinity."
  (cons (list (TypedVariable (Glob VAR)
                             (TypeSet (Type "WordNode")
                                      (Interval (Number LOWER) (Number UPPER)))))
        '()))

(define* (variable TERM #:optional (VAR (choose-var-name)))
  "Occurrence of a variable, where TERM can be a concept, a list of choices,
   or a wildcard."
  (cond ((equal? 'concept (car TERM))
         (concept (cdr TERM) VAR))
        ((equal? 'choices (car TERM))
         (choices (cdr TERM) VAR))
        ((equal? 'wildcard (car TERM))
         (wildcard (cadr TERM) (cddr TERM) VAR))))

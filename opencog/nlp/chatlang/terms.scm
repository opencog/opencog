;; ChatLang DSL for chat authoring rules
;;
;; Assorted functions for translating individual terms into Atomese fragments.

(use-modules (ice-9 optargs))

(define-public (word STR)
  "Literal word occurrence."
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         (c (list (ReferenceLink (Variable name) (WordNode STR))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define-public (lemma STR)
  "Lemma occurrence, aka canonical form of a term. This is the default
   for word mentions in the rule pattern."
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         ; Note: This converts STR to its lemma
         (c (list (LemmaLink (Variable name) (WordNode (get-lemma STR)))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define-public (phrase STR)
  "Occurrence of a phrase or a group of words.
   All the words are assumed to be literal / non-canonical."
  (fold (lambda (wd lst)
                (cons (append (car lst) (car wd))
                      (append (cdr lst) (cdr wd))))
        (cons '() '())
        (map word (string-split STR #\ ))))

(define*-public (concept STR #:optional (var (choose-var-name)))
  "Occurrence of a concept."
  (cons '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: chatlang-concept?")
                          (List (Glob var)
                                (Concept STR))))))

(define (terms-to-atomese TERMS)
  "Helper function to convert a list of terms into atomese.
   For use of choices and negation."
  (map (lambda (t) (cond ((equal? 'word (car t))
                          (Word (cdr t)))
                         ((equal? 'lemma (car t))
                          (Word (get-lemma (cdr t))))
                         ((equal? 'phrase (car t))
                          (List (map Word (string-split (cdr t) #\ ))))
                         ((equal? 'concept (car t))
                          (Concept (cdr t)))))
       TERMS))

(define*-public (choices TERMS #:optional (var (choose-var-name)))
  "Occurrence of a list of choices. Existence of either one of
   the words/lemmas/phrases/concepts in the list will be considered
   as a match."
  (cons '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: chatlang-choices?")
                          (List (Glob var)
                                (List (terms-to-atomese TERMS)))))))

(define*-public (unordered-matching TERMS #:optional (var (choose-var-name)))
  "Occurrence of a list of terms (words/lemmas/phrases/concepts)
   that can be matched in any orders."
  (fold (lambda (t lst)
                (cons (append (car lst) (car t))
                      (append (cdr lst) (cdr t))))
        (cons '() '())
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

(define-public (negation TERMS)
  "Absent of a term or a list of terms (words/lemmas/phrase/concepts)."
  (cons '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: chatlang-negation?")
                          (List (terms-to-atomese TERMS))))))

;; GHOST DSL for chat authoring rules
;;
;; Assorted functions for translating individual terms into Atomese fragments.

; ----------
; Helper functions

(define (genvar STR is-literal?)
  "Helper function for generating the name of a VariableNode,
   so as to make it slightly easier for a human to debug the code.
   is-literal? flag is for indicating whether this variable
   or glob is supposed to go to the word-seq or lemma-seq."
  (if is-literal?
      (string-append STR "-" (choose-var-name))
      (string-append (get-lemma STR) "-" (choose-var-name))))

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

; ----------
; The terms

(define (word STR)
  "Literal word occurrence."
  (let* ((v1 (WordNode STR))
         (v2 (Variable (genvar STR #f)))
         (l (WordNode (get-lemma STR)))
         (v (list (TypedVariable v2 (Type "WordInstanceNode"))))
         (c (list (WordInstanceLink v2 (Variable "$P"))
                  (ReferenceLink v2 v1))))
    (list v c (list v1) (list l))))

(define (lemma STR)
  "Lemma occurrence, aka canonical form of a term.
   This is the default for word mentions in the rule pattern."
  (let* ((v1 (Variable (genvar STR #t)))
         (v2 (Variable (genvar STR #f)))
         (l (WordNode (get-lemma STR)))
         (v (list (TypedVariable v1 (Type "WordNode"))
                  (TypedVariable v2 (Type "WordInstanceNode"))))
         ; Note: This converts STR to its lemma
         (c (list (ReferenceLink v2 v1)
                  (LemmaLink v2 l)
                  (WordInstanceLink v2 (Variable "$P")))))
    (list v c (list v1) (list l))))

(define (phrase STR)
  "Occurrence of a phrase or a group of words.
   All the words are assumed to be literal / non-canonical."
  (fold (lambda (wd lst)
                (list (append (car lst) (car wd))
                      (append (cadr lst) (cadr wd))
                      (append (caddr lst) (caddr wd))
                      (append (cadddr lst) (cadddr wd))))
        (list '() '() '() '())
        (map word (string-split STR #\sp))))

(define* (concept STR)
  "Occurrence of a concept."
  (let ((g1 (Glob (genvar STR #t)))
        (g2 (Glob (genvar STR #f)))
        (clength (concept-length (Concept STR))))
    (list (list (TypedVariable g1 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number clength))))
                (TypedVariable g2 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number clength)))))
          (list (Evaluation (GroundedPredicate "scm: ghost-concept?")
                            (List (Concept STR) g1)))
          (list g1)
          (list g2))))

(define* (choices TERMS)
  "Occurrence of a list of choices. Existence of either one of
   the words/lemmas/phrases/concepts in the list will be considered
   as a match."
  (let ((g1 (Glob (genvar "choices" #t)))
        (g2 (Glob (genvar "choices" #f)))
        (tlength (term-length TERMS)))
    (list (list (TypedVariable g1 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number tlength))))
                (TypedVariable g2 (TypeSet (Type "WordNode")
                                           (Interval (Number 1)
                                                     (Number tlength)))))
          (list (Evaluation (GroundedPredicate "scm: ghost-choices?")
                            (List (List (terms-to-atomese TERMS)) g1)))
          (list g1)
          (list g2))))

(define (negation TERMS)
  "Absent of a term or a list of terms (words/phrases/concepts)."
  (list '()  ; No variable declaration
        (list (Evaluation (GroundedPredicate "scm: ghost-negation?")
                          (List (terms-to-atomese TERMS))))
        ; Nothing for the word-seq and lemma-seq
        '() '()))

(define* (wildcard LOWER UPPER)
  "Occurrence of a wildcard that the number of atoms to be matched
   can be restricted.
   Note: -1 in the upper bound means infinity."
  (let* ((g1 (Glob (genvar "wildcard" #t)))
         (g2 (Glob (genvar "wildcard" #f))))
    (list (list (TypedVariable g1
                (TypeSet (Type "WordNode")
                         (Interval (Number LOWER) (Number UPPER))))
                (TypedVariable g2
                (TypeSet (Type "WordNode")
                         (Interval (Number LOWER) (Number UPPER)))))
        '()
        (list g1)
        (list g2))))

(define (variable VAR . GRD)
  "Occurence of a variable. The value grounded for it needs to be recorded.
   VAR is a number, it will be included as part of the variable name.
   GRD can either be a VariableNode or a GlobNode, which pass the actual
   value grounded in original words at runtime."
  (Evaluation (GroundedPredicate "scm: ghost-record-groundings")
    (List (List (ghost-var-word VAR) (List GRD))
          (List (ghost-var-lemma VAR)
                (ExecutionOutput (GroundedSchema "scm: ghost-get-lemma")
                                 (List GRD))))))

(define (context-function NAME ARGS)
  "Occurrence of a function in the context of a rule."
  (Evaluation (GroundedPredicate (string-append "scm: " NAME))
              (List ARGS)))

(define (action-function NAME ARGS)
  "Occurrence of a function in the action of a rule."
  (ExecutionOutput (GroundedSchema (string-append "scm: " NAME))
                   (List ARGS)))

(define (action-choices ACTIONS)
  "Pick one of the ACTIONS."
  (ExecutionOutput (GroundedSchema "scm: ghost-pick-action")
                   (Set ACTIONS)))

(define (get-var-words NUM)
  "Get the value grounded for a variable, in original words."
  (ExecutionOutput (GroundedSchema "scm: ghost-get-var-words")
                   (List (ghost-var-word NUM))))

(define (get-var-lemmas NUM)
  "Get the value grounded for a variable, in lemmas."
  (ExecutionOutput (GroundedSchema "scm: ghost-get-var-lemmas")
                   (List (ghost-var-lemma NUM))))

(define (get-user-variable UVAR)
  "Get the value of a user variable."
  (ExecutionOutput (GroundedSchema "scm: ghost-get-user-variable")
                   (List (ghost-uvar UVAR))))

(define (set-user-variable UVAR VAL)
  "Assign a string value to a user variable."
  (ExecutionOutput (GroundedSchema "scm: ghost-set-user-variable")
                   (List (ghost-uvar UVAR) VAL)))

(define (uvar-exist? UVAR)
  "Check if a user variable has been defined."
  (Evaluation (GroundedPredicate "scm: ghost-user-variable-exist?")
              (List (ghost-uvar UVAR))))

(define (uvar-equal? UVAR VAL)
  "Check if the value of the user variable VAR equals to VAL."
  ; TODO: VAL can also be a concept etc?
  (Evaluation (GroundedPredicate "scm: ghost-user-variable-equal?")
              (List (ghost-uvar UVAR) (Word VAL))))

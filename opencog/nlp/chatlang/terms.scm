;; ChatLang DSL for chat authoring rules
;;
;; Assorted functions for translating individual terms into Atomese fragments.
;; Each function returns a pair, where the car is the list of variables
;; specific to that term, and the cdr is the list of conditions that must be
;; true if the term is found in the input sentence.

(define (word w)
  "Literal word occurrence"
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         (c (list (ReferenceLink (Variable name) (WordNode w))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define (lemma w)
  "Lemma occurrence, aka canonical form of a term. This is the default
   for word mentions in the rule pattern."
  (let* ((name (choose-var-name))
         (v (list (TypedVariable (Variable name) (Type "WordInstanceNode"))))
         ; TODO -- need to actually get the lemma of "w" here
         (c (list (LemmaLink (Variable name) (WordNode w))
                  (WordInstanceLink (Variable name) (Variable "$P")))))
    (cons v c)))

(define (concept c)
  "Term is a member of a given concept, including concepts created via
   chat-concept"
  (let* ((var-wi (choose-var-name))
         (var-w (choose-var-name))
         (v (list (TypedVariable (Variable var-wi) (Type "WordInstanceNode"))
                  (TypedVariable (Variable var-w) (Type "WordNode"))))
         (c (list (ReferenceLink (Variable var-w) (Concept c))
                  (ReferenceLink (Variable var-wi) (Variable var-w))
                  (WordInstanceLink (Variable var-wi) (Variable "$P")))))
    (cons v c)))

(define (pos w p)
  "Term with a specific POS tag (note: canonical form of word, not literal)"
  (let* ((lemma-atomese (lemma w))
         (var-node (gar (caar lemma-atomese))))
    (cons (car lemma-atomese)
          (append (cdr lemma-atomese)
                  (list (PartOfSpeechLink var-node
                                          (DefinedLinguisticConcept p)))))))

(define (proper-names . w)
  "Terms represent multi-word proper names. It must have at least two words."
  (define w1-atoms (word (car w)))
  (define vars (car w1-atoms))
  (define conds (cdr w1-atoms))
  (define var-head (gar (caar w1-atoms)))
  (define (template w)
    (let* ((w-atoms (word w))
           (v (car w-atoms))
           (c (cdr w-atoms))
           (el (EvaluationLink (LinkGrammarRelationship "G")
                               (ListLink var-head
                                         (gar (car v))))))
      (append! vars v)
      (append! conds c (list el))
      (set! var-head (gar (car v)))))
  (for-each template (cdr w))
  (cons vars conds))

; This is NOT meant for external use
; The name of the variables ("$left_wall" and "$main_verb") are fixed
; here as they are shared with all the main subj, obj, and verb, and
; having fixed names seems to make it clearer and easier
(define* (main-verb-template #:optional cond-mv)
  "The template for generating the main verb atomese, since this is needed
   for defining main subject, verb, and object."
  (let* ((v (list (TypedVariable (Variable "$left_wall") (Type "WordInstanceNode"))
                  (TypedVariable (Variable "$main_verb") (Type "WordInstanceNode"))))
         (c (list (WordInstanceLink (Variable "$left_wall") (Variable "$P"))
                  (WordInstanceLink (Variable "$main_verb") (Variable "$P"))
                  (EvaluationLink (LinkGrammarRelationship "WV")
                                  (ListLink (Variable "$left_wall")
                                            (Variable "$main_verb"))))))
    (if cond-mv
      (cons v (append c cond-mv))
      (cons v c))))

; This is NOT meant for external use
; To generate the common atomese representation for main subject and object
(define (main-so-template w r)
  "The template for generating the main subject and object atomese."
  (let* ((main-verb-atomese (main-verb-template))
         (lemma-atomese (lemma w))
         (var-so (gar (caar lemma-atomese))))
    (cons (append (car main-verb-atomese)
                  (car lemma-atomese))
          (append (cdr main-verb-atomese)
                  (cdr lemma-atomese)
                  (list (EvaluationLink (DefinedLinguisticRelationship r)
                                        (ListLink (Variable "$main_verb")
                                                  var-so)))))))

(define (main-verb w)
  "Term is the main verb of the sentence."
  (main-verb-template
    ; TODO -- need to actually get the lemma of "w" here
    (list (LemmaLink (Variable "$main_verb") (Word w))
          (WordInstanceLink (Variable "$main_verb") (Variable "$P")))))

(define (main-subj w)
  "Term is the main subject of the sentence."
  (main-so-template w "_subj"))

(define (main-obj w)
  "Term is the main object of the sentence."
  (main-so-template w "_obj"))

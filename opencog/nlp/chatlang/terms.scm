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
  (let* ((var (choose-var-name))
         (v (list (TypedVariable (Variable var) (Type "WordInstanceNode"))))
         (c (list (Lemma (Variable var) (Word w))
                  (PartOfSpeech (Variable var) (DefinedLinguisticConcept p))
                  (WordInstanceLink (Variable var) (Variable "$P")))))
    (cons v c)))

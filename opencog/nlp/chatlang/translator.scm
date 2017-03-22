;; ChatLang DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
             (srfi srfi-1))

;; Shared variables for all terms
(define atomese-variable-template (list (TypedVariable (Variable "$S")
                                                       (Type "SentenceNode"))
                                        (TypedVariable (Variable "$P")
                                                       (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template (list (Parse (Variable "$P")
                                                (Variable "$S"))
                                         (State (Anchor "CurrentlyProcessing")
                                                (Variable "$S"))))

(define (process-pattern-term term atomese-pattern)
  "Process a single term -- calls the term function and appends the new
   variables and conditions to the existing pair."
  (let* ((atomese-for-term (primitive-eval term))
         (vars (append (car atomese-pattern) (car atomese-for-term)))
         (conds (append (cdr atomese-pattern) (cdr atomese-for-term))))
    (cons vars conds)))

; XXX TODO
(define (term-sequence-check terms)
  "Checks terms occur in the desired order. To be implemented."
  '())

; XXX TODO
(define (say text)
    (Node text)
)

(define yakking (psi-demand "Yakking" 0.9))

(define* (chat-rule pattern action #:optional name)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((template (cons atomese-variable-template atomese-condition-template))
         (proc-terms (fold process-pattern-term
                           template
                           pattern))
         (seq-check (term-sequence-check pattern)))

    (psi-rule
      (list (Satisfaction (VariableList (car proc-terms))
                          (And (append (cdr proc-terms) seq-check))))
      (primitive-eval action)
      (True)
      (stv .9 .9)
      yakking
      name)))

(define (member-words w)
  (let ((words (string-split w #\sp)))
    (if (= 1 (length words))
        (Word words)
        (List (map-in-order Word words)))))

(define (chat-concept name members)
  "Lets users create named concepts with explicit membership lists."
  (let* ((c (Concept name))
         (ref-members (append-map (lambda (m) (Reference (member-words m) c))
                                  members)))
    (List (cons c ref-members))))

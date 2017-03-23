;; ChatLang DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
             (opencog movement)
             (srfi srfi-1))

;; Shared variables for all terms
(define atomese-variable-template (list (TypedVariable (Variable "$S")
                                                       (Type "SentenceNode"))
                                        (TypedVariable (Variable "$P")
                                                       (Type "ParseNode"))))

;; Shared conditions for all terms
(define atomese-condition-template (list (Parse (Variable "$P")
                                                (Variable "$S"))
                                         (State (Anchor "Currently Processing")
                                                (Variable "$S"))))

(define (process-pattern-term term atomese-pattern)
  "Process a single term -- calls the term function and appends the new
   variables and conditions to the existing pair."
  (let* ((atomese-for-term (primitive-eval term))
         (vars (append (car atomese-pattern) (car atomese-for-term)))
         (conds (append (cdr atomese-pattern) (cdr atomese-for-term))))
    (cons vars conds)))

; XXX TODO FIXME
; This is a temporary hack for testing purpose
; Will think more on this and update it accordingly
(define (term-sequence-check terms)
  "Checks terms occur in the desired order. To be implemented."
  (define word-list
    (map (lambda (w) (cond ((equal? 'concept (car w)) (Glob (car (cdr w))))
                           (else (Word (car (cdr w))))))
         terms))
  (Evaluation (GroundedPredicate "scm: check-word-sequence")
              (List (Variable "$S")
                    (List word-list))))

(define (get-sent-words sent-node)
  (List (append-map
    (lambda (w)
      ; Ignore LEFT-WALL and punctuations
      (if (or (string-prefix? "LEFT-WALL" (cog-name w))
              (word-inst-match-pos? w "punctuation"))
          '()
          (cog-chase-link 'ReferenceLink 'WordNode w)))
    (car (sent-get-words-in-order sent-node)))))

(define (check-word-sequence sent-node word-list)
  (define result (cog-execute! (Map word-list (Set (get-sent-words sent-node)))))
  (if (not (null? (gar result)))
      (stv 1 1)
      (stv 0 1)))

(define (say text)
  (True (Put (DefinedPredicate "Say") (Node text))))

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
                          (And (append (cdr proc-terms) (list seq-check)))))
      (primitive-eval action)
      (True)
      (stv .9 .9)
      yakking
      name)))

(define (member-words w)
  (let ((words (string-split w #\sp)))
    (if (= 1 (length words))
        (Word (car words))
        (List (map-in-order Word words)))))

(define (chat-concept name members)
  "Lets users create named concepts with explicit membership lists."
  (let* ((c (Concept name))
         (ref-members (append-map (lambda (m) (list (Reference (member-words m) c)))
                                  members)))
    ref-members))

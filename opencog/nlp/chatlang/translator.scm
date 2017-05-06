;; ChatLang DSL for chat authoring rules
;;
;; A partial implementation of the top level translator that produces
;; PSI rules.
(use-modules (opencog)
             (opencog nlp)
             (opencog exec)
             (opencog openpsi)
             (opencog eva-behavior)
             (srfi srfi-1)
             (rnrs io ports)
             (ice-9 optargs))

(define (chatlang-prefix STR) (string-append "Chatlang: " STR))
(define chatlang-no-constant (Node (chatlang-prefix "No constant terms")))

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

(define (process-pattern-term TERM ATOMESE)
  "Process a single term -- calls the term function and appends the new
   variables and conditions to the existing pair."
  (define atomese-for-term
    (cond ((equal? 'lemma (car TERM))
           (lemma (cadr TERM)))
          ((equal? 'word (car TERM))
           (word (cadr TERM)))
          ((equal? 'phrase (car TERM))
           (phrase (cadr TERM)))
          ((equal? 'concept (car TERM))
           (concept (cadr TERM)))
          ; TODO
          (else (cons '() '()))))
  (define vars (append (car ATOMESE) (car atomese-for-term)))
  (define conds (append (cdr ATOMESE) (cdr atomese-for-term)))
  (cons vars conds))

(define yakking (psi-demand "Yakking" 0.9))

(define*-public (chat-rule PATTERN ACTION #:optional NAME)
  "Top level translation function. Pattern is a quoted list of terms,
   and action is a quoted list of actions or a single action."
  (let* ((template (cons atomese-variable-template atomese-condition-template))
         (proc-terms (fold process-pattern-term
                           template
                           pattern)))
    proc-terms))

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

(define-public (chatlang-concept? GLOB CONCEPT)
  "Check if the grounded value of the GlobNode is actually a member
   of the concept."
  (define grd (assoc-ref globs (cog-name GLOB)))
  (define membs (get-members CONCEPT))
  (if (not (equal? #f (member grd membs)))
      (stv 1 1)
      (stv 0 1)))

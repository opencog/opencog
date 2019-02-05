(use-modules (opencog exec))
(use-modules (opencog logger))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

;; Given an atom created with minsup-eval, get the pattern, texts and
;; ms
(define (get-pattern minsup-f)
  (cog-outgoing-atom (gdr minsup-f) 0))
(define (get-texts minsup-f)
  (cog-outgoing-atom (gdr minsup-f) 1))
(define (get-ms minsup-f)
  (cog-outgoing-atom (gdr minsup-f) 2))
(define (get-vardecl f)
  (cog-outgoing-atom f 0))
(define (get-body f)
  (cog-outgoing-atom f 1))

(define (abstraction-eval shabs-list minsup-g)
  (Evaluation
    (Predicate "abstraction")
    (List
      shabs-list
      minsup-g)))

(define (unary-conjunction-eval body)
  (Evaluation
    (GroundedPredicate "scm: unary-conjunction")
    ;; Wrap the single argument in List in case it is itself a list
    (List body)))

(define (unary-conjunction? body)
  (let ((body-type (cog-type body)))
    (and (not (equal? body-type 'PresentLink))
         (not (equal? body-type 'AndLink)))))

(define (unary-conjunction body)
  (bool->tv (unary-conjunction? body)))

(define (equal-top x)
  (Equal x (top)))

(define (not-equal-top x)
  (Not (equal-top x)))

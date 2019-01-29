(use-modules (opencog exec))
(use-modules (opencog logger))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

;; Given an atom created with minsup-eval, get the pattern, texts and
;; ms
;;
;; get-pattern and get-texts also work for isurp constructs
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

(define (unary-conjunction? body)
  (let ((body-type (cog-type body)))
    (and (not (equal? body-type 'PresentLink))
         (not (equal? body-type 'AndLink)))))

(define (unary-conjunction body)
  (bool->tv (unary-conjunction? body)))

(define (unary-conjunction-eval body)
  (Evaluation
    (GroundedPredicate "scm: unary-conjunction")
    ;; Wrap the single argument in List in case it is itself a list
    (List body)))

(define (unary-conjunction-pattern? pattern)
  (and (equal? (cog-type pattern) 'LambdaLink)
       (unary-conjunction? (get-body pattern))))

(define (unary-conjunction-pattern pattern)
  (bool->tv (unary-conjunction-pattern? pattern)))

(define (unary-conjunction-pattern-eval pattern)
  (Evaluation
    (GroundedPredicate "scm: unary-conjunction-pattern")
    pattern))

(define (equal-top x)
  (Equal x (top)))

(define (not-equal-top x)
  (Not (equal-top x)))

(define (powerset l)
  (if (null? l)
      '(())
      (let ((rst (powerset (cdr l))))
        (append (map (lambda (x) (cons (car l) x)) rst)
                rst))))

(define (copy-insert a l)
"
  Given `l`, a list of lists, insert `a` in each sublists `s` of `l`,
  and return the list of all modifications of `l`. For instance

  a = 4
  l = ((1) (2 3))

  return

  ( ((4 1) (2 3)) ((1) (4 2 3)) ((1) (2 3) (4)) )
"
  (if (null? l)
      (list (list (list a)))
      (let ((rst (map (lambda (x) (cons (car l) x)) (copy-insert a (cdr l))))
            (fst (cons (cons a (car l)) (cdr l))))
        (cons fst rst))))

(define (partitions l)
  (if (null? l)
      '(())
      (let* ((rst (partitions (cdr l)))
             (groups (map (lambda (x) (copy-insert (car l) x)) rst)))
        (concatenate groups))))

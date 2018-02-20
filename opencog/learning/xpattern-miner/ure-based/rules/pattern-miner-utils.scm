(use-modules (opencog exec))
(use-modules (opencog query))
(use-modules (opencog logger))
(use-modules (opencog rule-engine))
(use-modules (srfi srfi-1))

(cog-logger-set-level! "debug")
(cog-logger-set-stdout! #t)
(cog-logger-set-sync! #t)

(define (support pat texts ms)
"
  Return the min between the frequency of pat according to texts and
  ms, or #f if pat is ill-formed.
"
  ;; (cog-logger-debug "support pat = ~a, texts = ~a, ms = ~a" pat texts ms)
  (let* ((pat-prnx (cog-execute! pat))  ; get pat in prenex form
         (ill-formed (null? pat-prnx)))
    (if ill-formed
        #f
        (if (eq? (cog-type pat-prnx) 'LambdaLink)
            (let* ((texts-as (texts->atomspace texts))
                   (query-as (cog-new-atomspace texts-as))
                   (prev-as (cog-set-atomspace! query-as))
                   (bl (pattern->bindlink pat-prnx))
                   (results (cog-bind-first-n bl ms)))
              (cog-set-atomspace! prev-as)
              (cog-arity results))
            1))))

(define (get-members C)
"
  Given a concept node C, return all its members
"
  (let* ((member-links (cog-filter 'MemberLink (cog-incoming-set C)))
         (member-of-C (lambda (x) (equal? C (gdr x))))
         (members (map gar (filter member-of-C member-links))))
    members))

(define (size-ge texts ms)
  (let* ((result (>= (length (get-members texts)) (atom->number ms))))
    (bool->tv result)))

(define (texts->atomspace texts)
"
  Create an atomspace with all members of concept texts in it.
"
  (let* ((members (get-members texts))
         (texts-as (cog-new-atomspace)))
    (cog-cp members texts-as)
    texts-as))

(define (pattern->bindlink pattern)
"
  Turn a pattern into a BindLink for for subsequent pattern
  matching texts.
"
  (if (= (cog-arity pattern) 2)
      ;; With variable declaration
      (let* ((vardecl (gar pattern))
             (body (gdr pattern)))
        (Bind vardecl body body)) ; to deal with unordered links
      ;; Without variable declaration
      (let* ((body (gar pattern)))
        (Bind body body)))) ; to deal with unordered links

(define (replace-el l i v)
"
  Given a list, a zero-based index and a value, replace the element
  at the index by the value on a new list.
"
  (let* ((pre-i (take l i))
         (post-i (drop l (+ i 1))))
    (append pre-i (list v) post-i)))

(define (minsup pattern texts ms)
  (Evaluation
     (Predicate "minsup")
     (List
        pattern
        texts
        ms)))

(define (pattern-arity pattern)
"
  Return the arity of the pattern, or #f if it is ill-formed.
"
  (let* ((pat-prnx (cog-execute! pattern))  ; get pattern in prenex form
         (ill-formed (null? pat-prnx)))
    (if ill-formed
        #f
        (if (eq? (cog-type pat-prnx) 'LambdaLink)
            (length (cog-free-variables (cog-outgoing-atom pat-prnx 0)))
            0))))

(define (has-arity . args)
"
  Return (stv 1 1) if pattern (the first argument) has
  the given arity (the second argument), (stv 0 1) otherwise

  We do that because the type system doesn't allow to statically check
  that yet.
"
  (if (= (length args) 2)
      (let* ((pattern (list-ref args 0))
             (arity (inexact->exact (atom->number (list-ref args 1)))))
        (bool->tv (equal? (pattern-arity pattern) arity)))
      (stv 0 1)))

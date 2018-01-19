(define (support pat texts ms)
"
  Return the min between the frequency of pat according to texts and
  ms, or #f if pat is ill-formed.
"
  (let* ((pat-prnx (cog-execute! pat))  ; get pat in prenex form
         (ill-formed (null? pat-prnx)))
    (if ill-formed
        #f
        (let* ((texts-as (texts->atomspace texts))
               (query-as (cog-new-atomspace texts-as))
               (prev-as (cog-set-atomspace! query-as))
               (bl (pattern->bindlink pat-prnx))
               (results (cog-bind-first-n bl ms)))
          (cog-arity results)))))

(define (get-members C)
"
  Given a concept node C, return all its members
"
  (let* ((member-links (cog-filter 'MemberLink (cog-incoming-set C)))
         (member-of-C (lambda (x) (equal? C (gdr x))))
         (members (map gar (filter member-of-C member-links))))
    members))

(define (size-ge texts ms)
  (bool->tv (>= (length (get-members texts)) (atom->number ms))))

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

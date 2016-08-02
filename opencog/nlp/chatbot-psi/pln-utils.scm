(use-modules (opencog query))

(define (search-inferred-atoms)
  (let* (
        (ia-query (Get (State pln-inferred-atoms (Variable "$x"))))
        (ia (gar (cog-satisfying-set ia-query))))
    ia))

(define (get-inferred-atoms)
  (delete-duplicates (cog-get-all-nodes (search-inferred-atoms))))

(define (get-inferred-names)
  (get-names (get-inferred-atoms)))

(define (get-names atom-list)
  (map cog-name atom-list))

;; Insert the elements of a given Set to the elements of the Set
;; associated to the Anchor pln-inferred-atoms
(define (add-to-pln-inferred-atoms s)
  (let* ((sl (cog-outgoing-set s))
         (ia (cog-outgoing-set (search-inferred-atoms)))
         (slia (delete-duplicates (append sl ia))))
    (State pln-inferred-atoms (SetLink slia))))

;; Return a list of pairs (inferred atom, name list) 
(define (get-assoc-inferred-names)
  (let ((inferred-atoms-list (cog-outgoing-set (search-inferred-atoms)))
        (gen-assoc (lambda (x) (list x (get-names (cog-get-all-nodes x)))))
        )
    (map gen-assoc inferred-atoms-list)))

(define (search-input-utterance-words)
  (let* ((results (cog-chase-link 'StateLink 'ListLink input-utterance-words)))
    (if (null? results) '() (cog-outgoing-set (first results)))))

(define (get-input-utterance-names)
  (get-names (search-input-utterance-words)))

(define (get-last-sentence-id)
  (let* (
         (query (Bind
                 (VariableList
                  (Variable "$sentence")
                  (Variable "$parse"))
                 (And (State input-utterance-sentence (Variable "$sentence"))
                      (Parse (Variable "$parse") (Variable "$sentence")))
                 (Variable "$sentence")))
         (results (cog-outgoing-set (cog-bind query))))
    (if (null? results) '() (first results))))

(define (get-last-rec-id)
  (let* ((last-recognized-face (Anchor "last-recognized-face"))
         (results (cog-chase-link 'StateLink 'ConceptNode last-recognized-face)))
    (if (null? results) '() (first results))))

(define (shuffle l)
  (map cdr
    (sort (map (lambda (x) (cons (random 1.0) x)) l)
          (lambda (x y) (< (car x) (car y))))))

(define inference-results-key (Predicate "inference-results"))

(define (get-inferred-atoms trail)
    (define result (cog-value trail inference-results-key))
    (if (nil? result) (LinkValue) result)
)

(define (get-names atom-list)
  (map cog-name atom-list))

(define (add-to-pln-inferred-atoms trail inferences)
   (cog-set-value! trail inference-results-key inferences)
)

(define (search-input-utterance-words)
  (let* ((results (cog-chase-link 'StateLink 'ListLink input-utterance-words)))
    (if (nil? results) '() (cog-outgoing-set (first results)))))

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
         (results (cog-outgoing-set (cog-execute! query))))
    (if (nil? results) '() (first results))))

(define (get-last-rec-id)
  (let* ((last-recognized-face (Anchor "last-recognized-face"))
         (results (cog-chase-link 'StateLink 'ConceptNode last-recognized-face)))
    (if (nil? results) '() (first results))))

(define (shuffle l)
  (map cdr
    (sort
        (map (lambda (x) (cons (random 1.0 (random-state-from-platform)) x)) l)
         (lambda (x y) (< (car x) (car y))))))

;-------------------------------------------------------------------------------
;; Code to mock the HEAD. Given a person id and its name, creates the
;; call chat and create the following
;;
;; (Evaluation
;;    (Predicate "name")
;;    (List
;;       (Concept person-id)
;;       (Concept name)))
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept person-id)
;;       (Sentence <sentence-id>)))
(define (mock-HEAD-chat person-id name message)
  ;; Create name structure
  (Evaluation
     (Predicate "name")
     (List
        (Concept person-id)
        (Word name)))

  ;; Create say structure
  (let* ((sentence (ghost message)))
    (Evaluation
       (Predicate "say")
       (List
          (Concept person-id)
          sentence))))

;-------------------------------------------------------------------------------
;; Rule to put a name on the last sentence using:
;;
;; 1. The state of "last-recognized-face"
;;
;; (State
;;    (Anchor "last-recognized-face")
;;    (Concept <rec-id>))
;;
;; 2. The state of input-utterance-sentence
;;
;; (State
;;    input-utterance-sentence
;;    (Sentence <sentence-id>))
;;
;; To produce
;;
;; (Evaluation
;;    (Predicate "say")
;;    (List
;;       (Concept <rec-id>)
;;       (Sentence <sentence-1>)))
;;
;; In addition to that it also produces
;;
;; (Evaluation
;;    (Predicate "name")
;;    (List
;;       (Concept <rec-id>
;;       (Word <name>)))
;;
;; with specific ids and names.
(define (put-name-on-the-last-sentence)
  (let ((last-sentence-id (get-last-sentence-id))
        (last-rec-id (get-last-rec-id))
        ;; rec-ids and names
        (rec-id-1 "20839")
        (name-1 "Louis"))
    (cog-logger-info "[PLN-Reasoner] last-sentence-id = ~a" last-sentence-id)
    (cog-logger-info "[PLN-Reasoner] last-rec-id = ~a" last-rec-id)
    (StateLink (AnchorNode "last-recognized-face") (ConceptNode "20839"))
    (Evaluation
       (Predicate "name")
          (List
             (Concept rec-id-1)
             (Word name-1)))
    (if (or (nil? last-sentence-id) (nil? last-rec-id))
        '()
        (Evaluation
           (Predicate "say")
           (List
              last-rec-id
              last-sentence-id))))
)

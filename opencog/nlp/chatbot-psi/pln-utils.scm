(use-modules (opencog exec))

(load "states.scm")

(define (get-inferred-atoms)
    (let ((inferences (cog-execute!
            (Get (Member pln-inferred-atoms (Variable "$x"))))))

        (cog-outgoing-set inferences)
    )
)

(define (get-names atom-list)
  (map cog-name atom-list))

;; Insert the elements of a given Set to the elements of the Set
;; associated to the Anchor pln-inferred-atoms
(define (add-to-pln-inferred-atoms s)
; FIXME: Why is it not adding somethimes?
   (map (lambda (x) (Member pln-inferred-atoms x)) s)
)

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
  (chat message)
  (sleep 1)                             ; you never know

  ;; Create name structure
  (Evaluation
     (Predicate "name")
     (List
        (Concept person-id)
        (Word name)))

  ;; Create say structure
  (let* ((sentence (cog-chase-link 'ListLink 'SentenceNode (Node message))))
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
    (if (or (null? last-sentence-id) (null? last-rec-id))
        '()
        (Evaluation
           (Predicate "say")
           (List
              last-rec-id
              last-sentence-id))))
)

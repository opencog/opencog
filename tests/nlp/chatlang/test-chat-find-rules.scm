(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog exec))

(define anchor (Anchor "Currently Processing"))
(chat-concept "watch" (list "watching" "seeing"))

(define rule-1 (chat-rule '((proper-names "James" "Gilbert")
                            (word "liked")
                            (concept "watch")
                            (or-choices "movie" "TV"))
                          '(say "Not anymore?")))

(define rule-2 (chat-rule '((main-subj "Richard")
                            (main-verb "eat")
                            (pos "delicious" "adj")
                            (main-obj "fish"))
                          '(say "Me too")))

; Load the outputs of 'nlp-parse'
(load "test-chat-find-rules-atomese.scm")

; Get the SentenceNodes from structures like:
; (Evaluation (Predicate "sentence-rawtext")
;             (List (Sentence "sentence@1234")
;                   (Node "James Gilbert liked watching TV")))
(define sent-1 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "James Gilbert liked watching TV"))))

(define sent-2 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "Richard eats delicious fishes"))))

; Try to find the rules
(State anchor sent-1)
(define test-chat-find-rules-result-1
    (equal? (gar (chat-find-rules sent-1)) rule-1))

(State anchor sent-2)
(define test-chat-find-rules-result-2
    (equal? (gar (chat-find-rules sent-2)) rule-2))

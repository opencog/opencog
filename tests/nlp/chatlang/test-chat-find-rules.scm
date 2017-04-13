(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog exec))

(define anchor (Anchor "Currently Processing"))
(chat-concept "watch" (list "watching" "seeing"))
(chat-concept "drink" (list "drink" "consume" "sip"))
(chat-concept "coffee" (list "espresso" "latte" "cappuccino"))

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

(define rule-3 (chat-rule '((concept "drink")
                            (concept "coffee"))
                          '(say "nice")))

(define rule-4 (chat-rule '((word "I")
                            (lemma "like")
                            (unordered-matching "sleep" "eat")
                            (lemma "and")
                            (lemma "go")
                            (lemma "crazy"))
                          '(say "cool")))

; Load the outputs of 'nlp-parse'
(load "test-atomese.scm")

; Get the SentenceNodes from structures like:
; (Evaluation (Predicate "sentence-rawtext")
;             (List (Sentence "sentence@1234")
;                   (Node "James Gilbert liked watching TV")))
(define sent-1 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "James Gilbert liked watching TV"))))

(define sent-2 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "Richard eats delicious fishes"))))

(define sent-3 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "drink latte"))))

(define sent-4 (car (cog-chase-link 'ListLink 'SentenceNode
    (Node "I like eating, sleeping, and going crazy"))))

; Try to find the rules
(State anchor sent-1)
(define test-chat-find-rules-result-1
    (equal? (gar (chat-find-rules sent-1)) rule-1))

(State anchor sent-2)
(define test-chat-find-rules-result-2
    (equal? (gar (chat-find-rules sent-2)) rule-2))

(State anchor sent-3)
(define test-chat-find-rules-result-3
    (equal? (gar (chat-find-rules sent-3)) rule-3))

(State anchor sent-4)
(define test-chat-find-rules-result-4
    (equal? (gar (chat-find-rules sent-4)) rule-4))

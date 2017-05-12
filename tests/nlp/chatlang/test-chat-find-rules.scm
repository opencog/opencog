(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog exec))

; Utils
; -----
; Get the SentenceNodes from structures like:
; (Evaluation (Predicate "sentence-rawtext")
;             (List (Sentence "sentence@1234")
;                   (Node "this is a test")))
(define (get-sent-node TXT)
    (car (cog-chase-link 'ListLink 'SentenceNode (Node TXT))))

(define (chat TXT)
    (define sent (get-sent-node TXT))
    (State (Anchor "Chatlang: Currently Processing") sent)
    (chat-find-rules sent))

; The rules
; ---------
; Simplest case
(define rule-1 (cr '("hey") '("woah")))

; Phrase, lemma, and word
(define rule-2 (cr '("'John Smith' 'ate lunch") '("he did")))

; Concept
(create-concept "foo" "swallow")
(create-concept "eat" "ingest" "binge and purge" "~foo")
(define rule-3 (cr '("you ~eat food") '("of course")))

; Choices, and a rule with no constant
(define rule-4 (cr '("[hi hello]") '("nice to meet you")))
(define rule-5 (cr '("it says ['one two' ~foo]") '("I see")))

; Unordered matching
(define rule-6 (cr '("<<apples love oranges>> no") '("what?")))

; Sentence anchor
(define rule-7 (cr '("they like it < I think") '("good to know")))

; TODO: Negation

; The test
; --------
; Load the outputs of 'nlp-parse'
(load "test-atomese.scm")

; Try to find the rules
(define test-result-1 (equal? (gar (chat "hey")) rule-1))
(define test-result-2 (equal? (gar (chat "John Smith ate lunch")) rule-2))
(define test-result-3 (equal? (gar (chat "John Smith eats lunch")) '()))
(define test-result-4 (equal? (gar (chat "you ingest food")) rule-3))
(define test-result-5 (equal? (gar (chat "you binge and purge food")) rule-3))
(define test-result-6 (equal? (gar (chat "you swallow food")) rule-3))
(define test-result-7 (equal? (gar (chat "you eat food")) '()))
(define test-result-8 (equal? (gar (chat "hi")) rule-4))
(define test-result-9 (equal? (gar (chat "hello")) rule-4))
(define test-result-10 (equal? (gar (chat "it says one two")) rule-5))
(define test-result-11 (equal? (gar (chat "it says swallow")) rule-5))
(define test-result-12 (equal? (gar (chat "oranges love apples no?")) rule-6))
(define test-result-13 (equal? (gar (chat "I think they like it")) rule-7))

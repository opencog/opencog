(use-modules (opencog)
             (opencog nlp)
             (opencog nlp chatlang)
             (opencog exec))

(load "../../../opencog/nlp/chatlang/translator.scm")
(set! test-get-lemma #t)

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
(define rule-9 (cr '("he plays >") '("play what")))

; Negation
(create-concept "berry" "strawberry" "raspberry")
(define rule-8 (cr '("!['apple orange ~berry] I eat") '("me too")))

; Wildcard
(define rule-10 (cr '("I know *~3 nearby") '("let's go there")))
(define rule-11 (cr '("can *2 up") '("sure")))

; Variable
; (define rule-12 (cr '("his name is _*") '("hello _0")))
; (define rule-13 (cr '("who _~eat this") '("Bob '_0")))
; (define rule-14 (cr '("why did you _[jump run]") '("I like to _0")))

; The test
; --------
; Load the outputs of 'nlp-parse'
(load "test-atomese.scm")

; Try to find the rules
(define test-result-1 (equal? (gar (chat "hey")) (gdr rule-1)))
(define test-result-2 (equal? (gar (chat "John Smith ate lunch")) (gdr rule-2)))
(define test-result-3 (equal? (gar (chat "John Smith eats lunch")) '()))
(define test-result-4 (equal? (gar (chat "you ingest food")) (gdr rule-3)))
(define test-result-5 (equal? (gar (chat "you binge and purge food")) (gdr rule-3)))
(define test-result-6 (equal? (gar (chat "you swallow food")) (gdr rule-3)))
(define test-result-7 (equal? (gar (chat "you eat food")) '()))
(define test-result-8 (equal? (gar (chat "hi")) (gdr rule-4)))
(define test-result-9 (equal? (gar (chat "hello")) (gdr rule-4)))
(define test-result-10 (equal? (gar (chat "it says one two")) (gdr rule-5)))
(define test-result-11 (equal? (gar (chat "it says swallow")) (gdr rule-5)))
(define test-result-12 (equal? (gar (chat "oranges love apples no?")) (gdr rule-6)))
(define test-result-13 (equal? (gar (chat "I think they like it")) (gdr rule-7)))
(define test-result-14 (equal? (gar (chat "I eat apples")) (gdr rule-8)))
(define test-result-15 (equal? (gar (chat "I eat oranges")) '()))
(define test-result-16 (equal? (gar (chat "I eat raspberry")) '()))
(define test-result-17 (equal? (gar (chat "he plays")) (gdr rule-9)))
(define test-result-18 (equal? (gar (chat "he plays tennis")) '()))
(define test-result-19 (equal? (gar (chat "I know a good place nearby")) (gdr rule-10)))
(define test-result-20 (equal? (gar (chat "I know a really good restaurant nearby")) '()))
(define test-result-21 (equal? (gar (chat "can do sit up")) (gdr rule-11)))
(define test-result-22 (equal? (gar (chat "can you please speak up")) '()))

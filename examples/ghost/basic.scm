; A simple demo for GHOST
;
; NOTE: Make sure you have RelEx server started

(use-modules (opencog)
             (opencog nlp relex2logic)
             (opencog openpsi)
             (opencog eva-behavior)
             (opencog nlp ghost))

; -----
; A simple rule with only words/lemmas
(ghost-parse "u: (she ate apple) me too")

; To trigger the rule
; (test-ghost "she ate apples")

; -----
; Create a concept and a rule that contains it
(ghost-parse "concept: ~eat (eat ingest \"binge and purge\")")
(ghost-parse "u: (he eats chocolate) I like chocolates a lot!")

; To trigger the rule
; (test-ghost "he eats chocolates")

; -----
; Use of choice
(ghost-parse "u: (I can [read jump dance]) good to know")

; To trigger the rule, do either one of the below
; (test-ghost "I can read")
; (test-ghost "I can jump")
; (test-ghost "I can dance")

; -----
; Use of wildcard
(ghost-parse "u: (how * it) awesome")
(ghost-parse "u: (there *~2 cakes) I want them")

; To trigger the rules
; (test-ghost "how was it")
; (test-ghost "there are cakes")
; ... but not triggered by
; (test-ghost "there are three tiny cakes")

; -----
; Use of variables and user variables
(ghost-parse "u: (me name be _*) $name='_0 Hi '_0")
(ghost-parse "u: (what be me name) Your name is $name")

; To trigger the rule
; (test-ghost "my name is John")
; (test-ghost "what is my name")

; -----
; Use of negation
(ghost-parse "u: (!hate I * the pen) sure")

; To trigger the rule
; (test-ghost "I really like the pen")
; ... but not triggered by
; (test-ghost "I hate the pen")

; -----
; Use of sentence anchor
(ghost-parse "u: (< there be a cat) really")

; To trigger the rule
; (test-ghost "there is a cat in the park")
; ... but not triggered by
; (test-ghost "I think there is a cat")

; -----
; Use of function

(define-public (findkiller)
  ; ... the process of finding the killer ...
  ; The answer should be a list of nodes wrapped in a ListLink
  (List (Word "Bob") (Word "and") (Word "Alice")))

(Define
  (DefinedSchema "findkiller")
  (Lambda (ExecutionOutput (GroundedSchema "scm: findkiller") (List))))

(ghost-parse "u: (who killed the _*) I think ^findkiller killed the '_0")

; To trigger the rule
; (test-ghost "who killed the dinosaurs")

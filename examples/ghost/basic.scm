; A simple demo for GHOST
;
; NOTE: Make sure you have RelEx server started

(use-modules (opencog)
             (opencog nlp)
             (opencog nlp relex2logic)
             (opencog openpsi)
             (opencog ghost)
             (opencog ghost procedures))

; Disable the ECAN related config for this simple demo
(ghost-set-sti-weight 0)
(ghost-af-only #f)

; Start GHOST
(ghost-run)

; -----
; A simple rule with only words/lemmas
(ghost-parse "u: (she ate apple) me too")

; To trigger the rule
; (ghost "she ate apples")

; -----
; Create a concept and a rule that contains it
(ghost-parse "concept: ~eat (eat ingest \"binge and purge\")")
(ghost-parse "u: (~eat chocolate) I like chocolates a lot! ^keep()")

; To trigger the rule
; (ghost "he eats chocolates")
; (ghost "she ingests chocolates")
; (ghost "they binge and purge chocolates")

; -----
; Use of choice
(ghost-parse "u: (I can [read jump dance]) good to know ^keep()")

; To trigger the rule, do either one of the below
; (ghost "I can read")
; (ghost "I can jump")
; (ghost "I can dance")

; -----
; Use of wildcard
(ghost-parse "u: (how * it) awesome")
(ghost-parse "u: (there *~2 cakes) I want them ^keep()")

; To trigger the rules
; (ghost "how was it")
; (ghost "there are cakes")
; ... but not triggered by
; (ghost "there are three tiny cakes")

; -----
; Use of variables and user variables
(ghost-parse "u: (me name be _*) $name='_0 Hi '_0 ^keep()")
(ghost-parse "u: (what be me name) Your name is $name ^keep()")

; To trigger the rule
; (ghost "my name is John")
; (ghost "what is my name")

; -----
; Use of negation
(ghost-parse "u: (!hate I * the pen) sure ^keep()")

; To trigger the rule
; (ghost "I really like the pen")
; ... but not triggered by
; (ghost "I hate the pen")

; -----
; Use of sentence anchor
(ghost-parse "u: (< there be a cat) really ^keep()")

; To trigger the rule
; (ghost "there is a cat in the park")
; ... but not triggered by
; (ghost "I think there is a cat")

; -----
; Use of function

(define-public (findkiller)
  ; ... the process of finding the killer ...
  ; The answer should be a list of nodes wrapped in a ListLink
  (List (Word "Bob") (Word "and") (Word "Alice")))

(Define
  (DefinedSchema "findkiller")
  (Lambda (ExecutionOutput (GroundedSchema "scm: findkiller") (List))))

(ghost-parse "u: (who killed the _*) I think ^findkiller killed the '_0 ^keep()")

; To trigger the rule
; (ghost "who killed the dinosaurs")

; -----
; To exit GHOST
; (ghost-halt)

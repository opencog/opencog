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

; You can have a look at the GHOST status by doing
; (ghost-show-status)

; -----
; A simple rule with only words/lemmas
(ghost-parse "r: (she ate apple) me too")

; To trigger the rule
; (ghost "she ate apples")

; -----
; Create a concept and a rule that contains it
(ghost-parse "concept: ~eat (eat ingest \"binge and purge\")")
(ghost-parse "r: (~eat chocolate) I like chocolates a lot! ^keep()")

; To trigger the rule
; (ghost "he eats chocolates")
; (ghost "she ingests chocolates")
; (ghost "they binge and purge chocolates")

; -----
; Use of choice
(ghost-parse "r: (I can [read jump dance]) good to know ^keep()")

; To trigger the rule, do either one of the below
; (ghost "I can read")
; (ghost "I can jump")
; (ghost "I can dance")

; -----
; Use of wildcard
(ghost-parse "r: (how * it) awesome")
(ghost-parse "r: (there *~2 cakes) I want them ^keep()")

; To trigger the rules
; (ghost "how was it")
; (ghost "there are cakes")
; ... but not triggered by
; (ghost "there are three tiny cakes")

; -----
; Use of variables and user variables
(ghost-parse "r: (me name be _*) $name='_0 Hi '_0 ^keep()")
(ghost-parse "r: (what be me name) Your name is $name ^keep()")

; To trigger the rule
; (ghost "my name is John")
; (ghost "what is my name")

; -----
; Use of negation
(ghost-parse "r: (!hate I * the pen) sure ^keep()")

; To trigger the rule
; (ghost "I really like the pen")
; ... but not triggered by
; (ghost "I hate the pen")

; -----
; Use of sentence anchor
(ghost-parse "r: (< there be a cat) really ^keep()")

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

(ghost-parse "r: (who killed the _*) I think ^findkiller killed the '_0 ^keep()")

; To trigger the rule
; (ghost "who killed the dinosaurs")

; -----
; Use of rejoinder
(ghost-parse "
  r: (upgrade * phone) is it better now? ^keep()
    j1: (yes) that's great!
    j1: (no) that's awful!
")

; To trigger the rule
; (ghost "I've upgraded my phone")
; ... then either one of the below
; (ghost "yes")
; (ghost "no")

; -----
; Urge & Goal
; There are two goals and two sets of rules defined below, one of the rules in each set
; shares the same "context", i.e. can be triggered by the same textual input. The urges
; of the goals will influence which one get triggered

(ghost-parse "
  urge: (please_human=1 novelty=0)

  goal: (please_human=1)
    r: ORANGE_1 (orange) Squeezy! ^keep()
    r: MAN_DARK (man in the dark) Was it last night? ^increase_urge(novelty, 1) ^decrease_urge(please_human, 1) ^keep()

  goal: (novelty=1)
    r: ORANGE_2 (orange) A man with an orange? ^keep()
    r: NOTHING (nothing else) OK switching back ^increase_urge(please_human, 1) ^decrease_urge(novelty, 1) ^keep()
")

; To trigger the above rules, you may try this sequence:
; (ghost "he was holding an orange")
; (ghost "I saw a man in the dark")
; (ghost "he was holding an orange")
; (ghost "there was nothing else")

; -----
; To exit GHOST
; (ghost-halt)

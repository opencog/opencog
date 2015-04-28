; Sentences (as proposed by Ben):
; "The conference was slightly earlier than Hanukkah."
; "Hanukkah was during December."
; Question: "To what extent was the conference during December?"

; Links that RelEx2Logic should generate:
(BeforeLink (stv .4 .99)
    (ConceptNode "conference")
    (ConceptNode "Hanukkah")
)
(DuringLink (stv .9 .99)
    (ConceptNode "Hanukkah")
    (ConceptNode "December")
)

; Possible conclusion (via PLN + IA):
(DuringLink (stv .6 .7)
    (ConceptNode "conference")
    (ConceptNode "Hanukkah")
)

; Also investigate: given exact durations, e.g.
; 4 days of conference, 8 days of Hanukkah, 31 days of December,
; calculate relations more precisely

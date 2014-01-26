; A simple demo of multiple sources of evidence, to test that PLN can find them.

(define oc (ConceptNode "OpenCog" (stv 0.1 0.8)))
(define will_work (ConceptNode "will_work" (stv 0.1 0.8)))
(define ai_plan (ConceptNode "AI_plan" (stv 0.1 0.8)))

; Target
(define oc_will_work (InheritanceLink oc will_work))

; Firstly two things that the same Rule will find.
; This is semi-correct use of ImplicationLinks - they're not meant to use Nodes as premises, that would be for InhLinks usually
(define argument1 (ConceptNode "argument1" (stv 0.1 0.8)))
(define argument2 (ConceptNode "argument2" (stv 0.1 0.8)))

(ImplicationLink argument1 oc_will_work (stv 0.1 0.3))
(ImplicationLink argument2 oc_will_work (stv 0.75 0.3))

; This is correct use of InheritanceLinks
(InheritanceLink oc ai_plan (stv 0.3 0.7))
(InheritanceLink ai_plan will_work (stv 0.0 0.8))



; A simple demo of multiple sources of evidence, to test that PLN can find them.

(define oc (PredicateNode "OpenCog" (stv 0.1 0.8)))
(define will_work (PredicateNode "will_work" (stv 0.1 0.8)))
(define ai_plan (PredicateNode "AI_plan" (stv 0.1 0.8)))

; Target
(define oc_will_work (InheritanceLink oc will_work))

; Firstly two things that the same Rule will find.
(define argument1 (PredicateNode "argument1" (stv 0.1 0.8)))
(define argument2 (PredicateNode "argument2" (stv 0.1 0.8)))

(ImplicationLink argument1 oc_will_work (stv 0.1 0.3))
(ImplicationLink argument2 oc_will_work (stv 0.75 0.3))

(InheritanceLink oc ai_plan (stv 0.3 0.7))
(InheritanceLink ai_plan oc_will_work (stv 0.0 0.8))



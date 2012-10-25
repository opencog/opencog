; Joel is smart with degree .95 in general, then to what degree is he
; smart in the context of cheesemaking?

; facts
(define smart (ConceptNode "smart" (stv 0.2 0.5)))
(define Joel (ConceptNode "Joel" (stv 0.1 0.5)))
(define IJS (InheritanceLink (stv 0.95 0.9) Joel smart))
(define cheeseMaking (ConceptNode "cheeseMaking" (stv 0.1 0.7)))

; inference
(define target (ContextFreeToSensitiveRule cheeseMaking IJS))

; return the target, necessary so it can be automatically tested by
; PLNSchemeWrapperUTest.cxxtest
 target

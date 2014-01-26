; How much soreMuscles inherits from jumping in the context of Moon
(define moon (ConceptNode "moon" (stv 0.1 0.5)))
(define jumping (ConceptNode "jumping" (stv 0.2 0.5)))
(define soreMuscles (ConceptNode "soreMuscles" (stv 0.15 0.5)))

(define mAj (AndLink (stv -0.07 0.5) moon jumping))
(define mAs (AndLink (stv -0.04 0.5) moon soreMuscles))

; non-contextual form
(define ImAsmAj (InheritanceLink (stv 0.3 0.5) mAs mAj))

; inference to determine the TV of soreMuscles inherits from jumping
; in the context of Moon
(define target (ContextualizerRule ImAsmAj))

; return the target, necessary so it can be automatically tested by
; PLNSchemeWrapperUTest.cxxtest
target

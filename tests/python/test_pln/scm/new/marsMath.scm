; mathematics in the context of Mars
(define math (ConceptNode "math" (stv 0.9 0.5)))
(define mars (ConceptNode "mars" (stv 0.1 0.5)))

; how are the mathematics on Mars
(define SubsetMM (SubsetLink (stv 0.91 0.5) mars math))

; inference to determine the TV of mathematics in the context of Mars
(define target (ContextualizerRule SubsetMM))

; return the target, necessary so it can be automatically tested by
; PLNSchemeWrapperUTest.cxxtest
target

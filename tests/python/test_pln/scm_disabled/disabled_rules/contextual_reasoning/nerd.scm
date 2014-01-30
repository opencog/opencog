; in the context of mathematics a nerd is cool, because it is interesting
; in the context of making out a nerd is not cool, because it is not attractive

; primary concepts
(define cool (ConceptNode "cool" (stv 0.5 0.5)))
(define nerd (ConceptNode "nerd" (stv 0.5 0.5)))
(define interesting (ConceptNode "interesting" (stv 0.5 0.5)))
(define attractive (ConceptNode "attractive" (stv 0.5 0.5)))

; contexts
(define mathematics (ConceptNode "mathematics" (stv 0.5 0.5)))
(define makingOut (ConceptNode "makingOut" (stv 0.5 0.5)))

; contextualized concepts. Note that this can be infered using
; ContextFreeToSensitiveRule but that is not the purpose of that test
(define mathematicsCool (ContextLink (stv 0.5 0.5) mathematics cool))
(define mathematicsNerd (ContextLink (stv 0.5 0.5) mathematics nerd))
(define mathematicsInteresting (ContextLink (stv 0.5 0.5) mathematics interesting))
(define mathematicsAttractive (ContextLink (stv 0.5 0.5) mathematics attractive))
(define makingOutCool (ContextLink (stv 0.5 0.5) makingOut cool))
(define makingOutNerd (ContextLink (stv 0.5 0.5) makingOut nerd))
(define makingOutInteresting (ContextLink (stv 0.5 0.5) makingOut interesting))
(define makingOutAttractive (ContextLink (stv 0.5 0.5) makingOut attractive))

; inheritances
(define IC (InheritanceLink (stv 0.5 0.5) interesting cool))
(define NI (InheritanceLink (stv 0.5 0.5) nerd interesting))
(define AC (InheritanceLink (stv 0.5 0.5) attractive cool))
(define NA (InheritanceLink (stv 0.5 0.5) nerd attractive))

; contextual inheritances
(define mathematicsIC (ContextLink (stv 0.9 0.8) mathematics IC))
(define mathematicsNI (ContextLink (stv 0.7 0.7) mathematics NI))
(define makingOutAC (ContextLink (stv 0.9 0.6) makingOut AC))
(define makingOutNA (ContextLink (stv 0.4 0.7) makingOut NA))

;
; Inference
;
; a nerd is cool in the context of mathematics
(define target1 (InheritanceDeductionRule NI IC mathematics))
; a nerd is not cool in the context of makingOut
(define target2 (InheritanceDeductionRule NA AC makingOut))

; return the target, necessary so it can be automatically tested by
; PLNSchemeWrapperUTest.cxxtest
target1

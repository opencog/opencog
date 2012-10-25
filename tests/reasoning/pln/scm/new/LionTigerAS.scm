;scm
;
; LionTigerKB.scm - contains Subset links between concept Lion (resp. Tiger)
; and properties like Mammal, Carnivor, Striped, Predator, FacingExtincion
; as well as NOT Lion (resp. NOT Tiger) and the same properties.
; It is used to test IntensionalInhertianceRule
;
; define concepts. Note that PLN ignores atoms with (0,0) truth values.
(define lion (ConceptNode "Lion" (stv 0.5 0.999)))
(define tiger (ConceptNode "Tiger" (stv 0.5 0.999)))
; define properties
(define mammal (ConceptNode "Mammal"))
(define carnivor (ConceptNode "Carnivor"))
(define striped (ConceptNode "Striped"))
(define predator (ConceptNode "Predator"))
(define facingExtincion (ConceptNode "FacingExtincion"))
; define default confidence
(define dc 0.5)
; Properties of Lion
(SubsetLink (stv 1 dc) lion mammal)
(SubsetLink (stv 1 dc) lion carnivor)
(SubsetLink (stv 0 dc) lion striped)
(SubsetLink (stv 1 dc) lion predator)
(SubsetLink (stv 0.8 dc) lion facingExtincion)
; Properties of NOT Lion
(SubsetLink (stv 0.2 dc) (NotLink lion) mammal)
(SubsetLink (stv 0.1 dc) (NotLink lion) carnivor)
(SubsetLink (stv 0.17 dc) (NotLink lion) striped)
(SubsetLink (stv 0.3 dc) (NotLink lion) predator)
(SubsetLink (stv 0.4 dc) (NotLink lion) facingExtincion)
; Properties of Tiger
(SubsetLink (stv 1 dc) tiger mammal)
(SubsetLink (stv 1 dc) tiger carnivor)
(SubsetLink (stv 0.9 dc) tiger striped)
(SubsetLink (stv 1 dc) tiger predator)
(SubsetLink (stv 1 dc) tiger facingExtincion)
; Properties of NOT Tiger
(SubsetLink (stv 0.2 dc) (NotLink tiger) mammal)
(SubsetLink (stv 0.1 dc) (NotLink tiger) carnivor)
(SubsetLink (stv 0.15 dc) (NotLink tiger) striped)
(SubsetLink (stv 0.3 dc) (NotLink tiger) predator)
(SubsetLink (stv 0.37 dc) (NotLink tiger) facingExtincion)



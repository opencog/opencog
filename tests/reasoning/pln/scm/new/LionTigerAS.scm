;scm
;
; LionTigerKB.scm - contains Subset links between concept Lion (resp. Tiger)
; and properties like Mammal, Carnivor, Striped, Predator, FacingExtincion
; as well as the negated versions of those properties.
; It is used to test IntensionalInhertianceRule
; "mammal" is a distinctive property of "Lion" if:
;   x being a mammal makes it more likely x is a lion.
;   i.e. P(x is a lion | x is a mammal) > P(x is a lion | x is not a mammal)
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
; How likely various things are to be a Lion
(SubsetLink (stv 1 dc) mammal lion)
(SubsetLink (stv 1 dc) carnivor lion)
(SubsetLink (stv 0 dc) striped lion)
(SubsetLink (stv 1 dc) predator lion)
(SubsetLink (stv 0.8 dc) facingExtincion lion)
; How likely you are to be a Lion if you are NOT those things
(SubsetLink (stv 0.2 dc) (NotLink mammal) lion)
(SubsetLink (stv 0.1 dc) (NotLink carnivor) lion)
(SubsetLink (stv 0.17 dc) (NotLink striped) lion)
(SubsetLink (stv 0.3 dc) (NotLink predator) lion)
(SubsetLink (stv 0.4 dc) (NotLink facingExtincion) lion)

(SubsetLink (stv 1 dc) mammal tiger)
(SubsetLink (stv 1 dc) carnivor tiger)
(SubsetLink (stv 0.9 dc) striped tiger)
(SubsetLink (stv 1 dc) predator tiger)
(SubsetLink (stv 1 dc) facingExtincion tiger)

(SubsetLink (stv 0.2 dc) (NotLink mammal) tiger)
(SubsetLink (stv 0.1 dc) (NotLink carnivor) tiger)
(SubsetLink (stv 0.15 dc) (NotLink striped) tiger)
(SubsetLink (stv 0.3 dc) (NotLink predator) tiger)
(SubsetLink (stv 0.37 dc) (NotLink facingExtincion) tiger)



;scm
;
; LionTigerKB.scm - contains Subset links between concept Lion (resp. Tiger)
; and properties like Mammal, Carnivor, Striped, Predator, FacingExtincion
; as well as NOT Lion (resp. NOT Tiger) and the same properties.
; It is used to test IntensionalInhertianceRule
;
; Properties of Lion
(SubsetLink (stv 1 0) (ConceptNode "Lion") (ConceptNode "Mammal"))
(SubsetLink (stv 1 0) (ConceptNode "Lion") (ConceptNode "Carnivor"))
(SubsetLink (stv 0 0) (ConceptNode "Lion") (ConceptNode "Striped"))
(SubsetLink (stv 1 0) (ConceptNode "Lion") (ConceptNode "Predator"))
(SubsetLink (stv 0.8 0) (ConceptNode "Lion") (ConceptNode "FacingExtincion"))
; Properties of NOT Lion
(SubsetLink (stv 0.2 0) (NotLink (ConceptNode "Lion")) (ConceptNode "Mammal"))
(SubsetLink (stv 0.1 0) (NotLink (ConceptNode "Lion")) (ConceptNode "Carnivor"))
(SubsetLink (stv 0.17 0) (NotLink (ConceptNode "Lion")) (ConceptNode "Striped"))
(SubsetLink (stv 0.3 0) (NotLink (ConceptNode "Lion")) (ConceptNode "Predator"))
(SubsetLink (stv 0.4 0) (NotLink (ConceptNode "Lion")) (ConceptNode "FacingExtincion"))
; Properties of Tiger
(SubsetLink (stv 1 0) (ConceptNode "Tiger") (ConceptNode "Mammal"))
(SubsetLink (stv 1 0) (ConceptNode "Tiger") (ConceptNode "Carnivor"))
(SubsetLink (stv 0.9 0) (ConceptNode "Tiger") (ConceptNode "Striped"))
(SubsetLink (stv 1 0) (ConceptNode "Tiger") (ConceptNode "Predator"))
(SubsetLink (stv 1 0) (ConceptNode "Tiger") (ConceptNode "FacingExtincion"))
; Properties of NOT Tiger
(SubsetLink (stv 0.2 0) (NotLink (ConceptNode "Tiger")) (ConceptNode "Mammal"))
(SubsetLink (stv 0.1 0) (NotLink (ConceptNode "Tiger")) (ConceptNode "Carnivor"))
(SubsetLink (stv 0.15 0) (NotLink (ConceptNode "Tiger")) (ConceptNode "Striped"))
(SubsetLink (stv 0.3 0) (NotLink (ConceptNode "Tiger")) (ConceptNode "Predator"))
(SubsetLink (stv 0.37 0) (NotLink (ConceptNode "Tiger")) (ConceptNode "FacingExtincion"))

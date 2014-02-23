; test to check if the ContextPredToSat in conjunction with
; Decontextualizer rule works

(define Music (ConceptNode "Music"))
(define isCat (PredicateNode "isCat" (stv 0.1 0.3)))

; fact
(define CMC (ContextLink (stv 0.7 0.8) Music isCat))

; inference, turn the contextLink into SubsetLink
(define target (DecontextualizerRule CMC))

; this is so PLNSchemeWrapperUTest can test that inference
target

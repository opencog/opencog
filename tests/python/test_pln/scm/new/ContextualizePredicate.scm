; test to check if the ContextPredToSat in conjunction with
; Decontextualizer rule works

(define Music (ConceptNode "Music"))
(define catTV (stv 0.1 0.3))
(define isCat (PredicateNode "isCat" catTV))
(define Cat (SatisfyingSetLink catTV isCat))

; fact
(define CMC (SubsetLink (stv 0.7 0.8) Music Cat))

; inference, turn the contextLink into SubsetLink
(define target (ContextualizerRule CMC))

; this is so PLNSchemeWrapperUTest can test that inference
target

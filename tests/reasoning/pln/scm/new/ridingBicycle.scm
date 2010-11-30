; PLN Scheme wrapper, for building forward inference step-by-step 
; example to proof "MoveForward" given
; 1) in the context "RidingBicycle", "Pedaling" implies "MoveForward"
; 2) "RidingBicycle"
(define RB (ConceptNode "RindingBicycle" (stv 1.0 1.0)))
(define Ped (ConceptNode "Pedaling" (stv 7.0 5.0)))
(define MF (ConceptNode "MovingForward"))
(define PedImpliesMF (ImplicationLink (stv 0.2 0.7) Ped MF))
(define CXPedImpliesMF (ContextLink (stv 0.8 0.9) RB PedImpliesMF))


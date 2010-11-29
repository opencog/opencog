; PLN Scheme wrapper, for building forward inference step-by-step 
; example to proof "MoveForward" given
; 1) in the context "RidingBicycle", "Pedaling" implies "MoveForward"
; 2) "RidingBicycle"
(define RB (ConceptNode "RindingBicycle"))
(define Ped (ConceptNode "Pedaling"))
(define MF (ConceptNode "MovingForward"))
(define PedImpliesMF (ImplicationLink (stv 0.2 0.7) Ped MF))
(define CXPedImpliesMF (ContextLink (stv 0.8 0.9) RB PedImpliesMF))

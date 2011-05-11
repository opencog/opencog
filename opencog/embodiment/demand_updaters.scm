;
; Demand updaters
;
; @author Zhenhua Cai czhedu@gmail.com
; @date   2011-05-09
;

; The script below is only for test. 
;
; For real OpenPsi, it would be much more complex. Such as the example provided by Nil:
;
;     EnergyDemandSchema = (FoodIntake - MovementAmount) + EnergyDemandValue * 0.9
;
; Where EnergyDemandValue is obviously the value of EnergyDemandSchema 
; calculated at the previous cycle
;

(define (EnergyDemandUpdater)
    (random:uniform) 
) 

(define (WaterDemandUpdater)
    (random:uniform) 
)    

(define (IntegrityDemandUpdater)
    (random:uniform) 
)

(define (AffiliationDemandUpdater)
    (random:uniform) 
)

(define (CertaintyDemandUpdater)
     (random:uniform) 
) 

(define (CompetenceDemandUpdater)
      (random:uniform) 
)

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define (TestEnergyDemandUpdater)
    (random:uniform) 
)


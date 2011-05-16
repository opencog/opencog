;
; Demand updaters
;
; @author Zhenhua Cai czhedu@gmail.com
; @date   2011-05-16
;

(define (EnergyDemandUpdater)
    (get_latest_predicate_truth_value_mean "energy") 
) 

(define (WaterDemandUpdater)
    (- 1
       (get_latest_predicate_truth_value_mean "thirst") 
    ) 
)    

(define (IntegrityDemandUpdater)
    (get_latest_predicate_truth_value_mean "fitness")
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


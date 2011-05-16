;
; Modulator updaters
;
; @author Zhenhua Cai czhedu@gmail.com
; @date   2011-05-11
;

; I borrow a few equations from the paper below
; Psi and MicroPsi A Novel Approach to Modeling Emotion and Cognition in a Cognitive Architecture, 
; Joscha Bach, Dietrich Dörner, Ronnie Vuine

; Activation controls the agent's readiness for action. 
; The higher the activation, the pressing it has become to react to the situation at hand,
; and faster decisions are sought. 
; A higher activation would lead to few details and less schematic depth. 
;

(define (ActivationModulatorUpdater)
    (* (get_predicate_truth_value_mean "CertaintyDemandGoal")

       (- 1
          (expt (get_predicate_truth_value_mean "CompetenceDemandGoal") 0.5)  
       )
    )
)

; Resolution level affects perception. 
; The range of resolution level is [0, 1]
; A low resolution level tends to miss differences, then the agent would get a better overview

(define (ResolutionModulatorUpdater)
    (- 1
       (expt (get_latest_modulator_or_demand_value "ActivationModulator") 0.5)
    ) 
)

; The frequency of the securing behavior is inversily determined by SecuringThreshold.
; The range of securing threshold is [0, 1]
; The value of securing threshold is proportional to the strength of the current motive, 
; i.e. in the face of urgency, there will be less orientation. 
; It also depends on the uncertainty in the current context.
; An undetermined environment requires more orientation, i.e. lower securing threshold. 

(define (SecuringThresholdModulatorUpdater)
    (normalize (/ (+ (get_predicate_truth_value_mean "CertaintyDemandGoal") 1)
                  (+ (get_predicate_truth_value_mean "CurrentDemandGoal") 1)
               )

               0.5
               2
    )
)

; SelectionThreshold is a bias added to the strength of the currently selected motive (Demand Goal). 
; The range of SelectionThreshold is [0, 1]
; A higher selection threshold leads to "stubbornness", makes it harder to switch motives (Demand Goals), 
; then oscillations can be avoided.
; While a lower one results in opportunism/flexibility, or even motive fluttering. 

(define (SelectionThresholdModulatorUpdater)
    (clip_within (* (+ (get_latest_modulator_or_demand_value "SelectionThresholdModulator") 0.5)
                    (+ (get_latest_modulator_or_demand_value "ActivationModulator") 0.5)
                 )

                 0.001
                 1
    )
)


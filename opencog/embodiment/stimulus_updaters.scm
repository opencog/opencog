;
; Stimulus updaters
;
; @author Troy Huang <huangdeheng@gmail.com>
; @date   2011-12-19
;

; This module borrows idea from Jinhua's modulator updater module.

; Activation controls the agent's readiness for action. 
; The higher the activation, the pressing it has become to react to the situation at hand,
; and faster decisions are sought. 
; A higher activation would lead to few details and less schematic depth. 
;

(define (ActivationStimulusUpdater)
;    (* (get_truth_value_mean (cog-tv CertaintyDemandGoal))
;           (expt (get_truth_value_mean (cog-tv CompetenceDemandGoal)) 0.5)
;    )       
    (let ( (latest_intensity (get_predicate_truth_value_mean "ActivationStimulus"))
         )
         ; TODO: finish the formulas once stimulus can be applied to affect modulator.
         (+ 0.5
             (* 
                (- latest_intensity 0.5)
                0.95
             )
         )
    )
)

; Resolution level affects perception. 
; The range of resolution level is [0, 1]
; A low resolution level tends to miss differences, then the agent would get a better overview

(define (ResolutionStimulusUpdater)
    (let ( (latest_intensity (get_predicate_truth_value_mean "ResolutionStimulus"))
         )

        (+ 0.5
             (* 
                 (- latest_intensity 0.5)
                 0.9
             )
        )
    ) 
)

; The frequency of the securing behavior is inversily determined by SecuringThreshold.
; The range of securing threshold is [0, 1]
; The value of securing threshold is proportional to the strength of the current motive, 
; i.e. in the face of urgency, there will be less orientation. 
; It also depends on the uncertainty in the current context.
; An undetermined environment requires more orientation, i.e. lower securing threshold. 
; Finally, it relates with Integrity. When the agetn gets hurt (lower Integriry), it
; concerns more about background changes, that is lower securing threshold. 

(define (SecuringThresholdStimulusUpdater)
;    (* (expt (get_truth_value_mean (cog-tv CertaintyDemandGoal)) 0.5)
;       (expt (get_truth_value_mean (cog-tv IntegrityDemandGoal)) 2)
;    )
    (let ( (latest_intensity (get_predicate_truth_value_mean "SecuringThresholdStimulus"))
         )
;         (* (/ certainty (+ certainty 0.05) )
;            (expt integrity 3)
;         )
         (+ 0.5
             (* 
                 (- latest_intensity 0.5)
                 0.93
             )
         )
    )
)

; SelectionThreshold is a bias added to the strength of the currently selected motive (Demand Goal). 
; The range of SelectionThreshold is [0, 1]
; A higher selection threshold leads to "stubbornness", makes it harder to switch motives (Demand Goals), 
; then oscillations can be avoided.
; While a lower one results in opportunism/flexibility, or even motive fluttering. 
; Lower competence causes lower selection threshold, that is tend to more random behaviour. 

(define (SelectionThresholdStimulusUpdater)
;    (clip_within (* (+ (get_truth_value_mean (cog-tv CompetenceDemandGoal)) 0.1)
;                    (+ (get_latest_predicate_truth_value_mean "ActivationModulator") 0.3)
;                 )
;
;                 0.001
;                 1
;    )

;    (let ( (competence (get_truth_value_mean (cog-tv CompetenceDemandGoal)) )
;           (activation (get_latest_predicate_truth_value_mean "ActivationModulator") )
;         )
;         (* (/ activation (+ activation 0.05) )
;            (expt competence 2)
;         )
;    )

    (let ( (latest_intensity (get_predicate_truth_value_mean "SelectionThresholdStimulus"))
         )

         (+ 0.5
             (* 
                 (- latest_intensity 0.5)
                 0.95
             )
         )
    )
)

; Pleasure level stimulus. 

(define (PleasureStimulusUpdater)
    (let
       ( (latest_intensity (get_predicate_truth_value_mean "PleasureStimulus"))
       )

       (+ 0.5
             (* 
                 (- latest_intensity 0.5)
                 0.85
             )
       )
    ) 
)

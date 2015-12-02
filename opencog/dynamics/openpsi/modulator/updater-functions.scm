;
; Modulator updaters
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-12-09
;
; @update Troy Huang <huangdeheng@gmail.com> 2011-12-19
;

; I borrow a few equations from the paper below
; Psi and MicroPsi A Novel Approach to Modeling Emotion and Cognition in a Cognitive Architecture,
; Joscha Bach, Dietrich Dörner, Ronnie Vuine

; Activation controls the agent's readiness for action.
; The higher the activation, the pressing it has become to react to the situation at hand,
; and faster decisions are sought.
; A higher activation would lead to few details and less schematic depth.
;

; This replaces what was done by the different StimulusUpdaterAgent
; If the factor is < 1 then the modulator values will continuesely decrease.
; XXX: What is the criteria for choosing the default factor value??
(define (stimulate latest-modulator-value factor)
    (+ 0.5
        (*
            (- latest-modulator-value 0.5)
            factor
        )
    )
)

(define (ActivationModulatorUpdater)
;    (* (get_truth_value_mean (cog-tv CertaintyDemandGoal))
;           (expt (get_truth_value_mean (cog-tv CompetenceDemandGoal)) 0.5)
;    )
    (let ((competence (cog-tv-mean (ConceptNode "OpenPsi: Competence")))
           (energy (cog-tv-mean (ConceptNode "OpenPsi: Energy")))
           (latest-value (cog-tv-mean (ConceptNode "OpenPsi: Activation")))
           )

         (+  (* 2 (stimulate latest-value 0.95))
             (* (/ competence (+ competence 0.5) )
                (/ energy (+ 0.05 energy) )
             )
         )
    )
)

; Resolution level affects perception.
; The range of resolution level is [0, 1]
; A low resolution level tends to miss differences, then the agent would get a better overview

(define (ResolutionModulatorUpdater)
    (let ((latest-value (cog-tv-mean (ConceptNode "OpenPsi: Resolution")))
          (activation (cog-tv-mean (ConceptNode "OpenPsi: Activation")))
         )

        (+  (* 2 (stimulate latest-value 0.9))
            (- 1
               (expt activation 0.5)
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

(define (SecuringThresholdModulatorUpdater)
;    (* (expt (get_truth_value_mean (cog-tv CertaintyDemandGoal)) 0.5)
;       (expt (get_truth_value_mean (cog-tv IntegrityDemandGoal)) 2)
;    )
    (let ( (certainty (cog-tv-mean (ConceptNode "OpenPsi: Certainty")))
           (integrity (cog-tv-mean (ConceptNode "OpenPsi: Integrity")))
           (latest-value (cog-tv-mean
                (ConceptNode "OpenPsi: SecuringThreshold")))
         )
;         (* (/ certainty (+ certainty 0.05) )
;            (expt integrity 3)
;         )
         (+  (* 2 (stimulate latest-value 0.93))
             (expt integrity 3)
         )
    )
)

; SelectionThreshold is a bias added to the strength of the currently selected motive (Demand Goal).
; The range of SelectionThreshold is [0, 1]
; A higher selection threshold leads to "stubbornness", makes it harder to switch motives (Demand Goals),
; then oscillations can be avoided.
; While a lower one results in opportunism/flexibility, or even motive fluttering.
; Lower competence causes lower selection threshold, that is tend to more random behaviour.

(define (SelectionThresholdModulatorUpdater)
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

    (let ( (competence (cog-tv-mean (ConceptNode "OpenPsi: Competence")))
           (latest-value (cog-tv-mean
                (ConceptNode "OpenPsi: SelectionThreshold")))
         )

         (+  (* 2 (stimulate latest-value 0.95))
             (fuzzy_equal competence 1 15)
         )
    )
)

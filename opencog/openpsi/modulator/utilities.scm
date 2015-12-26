; Copyright (C) 2015 OpenCog Foundation
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

(use-modules (srfi srfi-1))   ; For `remove!`

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
    (let ((competence (tv-mean (cog-tv (ConceptNode "OpenPsi: Competence"))))
           (energy (tv-mean (cog-tv (ConceptNode "OpenPsi: Energy"))))
           (latest-value (tv-mean (cog-tv (ConceptNode "OpenPsi: Activation"))))
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
    (let ((latest-value (tv-mean (cog-tv (ConceptNode "OpenPsi: Resolution"))))
          (activation (tv-mean (cog-tv (ConceptNode "OpenPsi: Activation"))))
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
    (let ( (certainty (tv-mean (cog-tv (ConceptNode "OpenPsi: Certainty"))))
           (integrity (tv-mean (cog-tv (ConceptNode "OpenPsi: Integrity"))))
           (latest-value (tv-mean (cog-tv
                (ConceptNode "OpenPsi: SecuringThreshold"))))
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

    (let ( (competence (tv-mean (cog-tv (ConceptNode "OpenPsi: Competence"))))
           (latest-value (tv-mean (cog-tv
                (ConceptNode "OpenPsi: SelectionThreshold"))))
         )

         (+  (* 2 (stimulate latest-value 0.95))
             (fuzzy_equal competence 1 15)
         )
    )
)

(define (psi-get-modulators)
"
  Returns a list containging the atoms defining the modulators.
"
    (let* ((psi-modulator (ConceptNode
                            (string-append (psi-prefix-str) "Modulator")))
          (inheritance-list (cog-incoming-set psi-modulator))
         )
         (remove!
             (lambda (x) (equal? (cog-type x) 'VariableNode))
             (par-map cog-get-partner inheritance-list
                 (make-list (length inheritance-list) psi-modulator))
         )
    )
)

; --------------------------------------------------------------
(define (define-psi-modulator modulator-name stimulus-value)
"
  Define an OpenPsi modulator. By default an in-born stimulus is defined.

  modulator-name:
    - The name of the modulator that is created.

  stimulus-value:
    - The initial stimulus level. This is the strength of the modulator's
      ConceptNode stv. The confidence of the stv is always 1.
"
    (let ((modulator (string-append (psi-prefix-str) modulator-name)))
        (list
            (InheritanceLink
                (ConceptNode modulator (stv stimulus-value 1))
                (ConceptNode (string-append (psi-prefix-str) "Modulator"))
            )

            ; This is the default function that each psi-modulator must have.
            (EvaluationLink
                (PredicateNode (string-append (psi-prefix-str) "Stimulus"))
                (ListLink
                    (GroundedSchemaNode "scm: psi-modulator-updater")
                    (ConceptNode modulator)
                )
            )
        )
    )
)

; --------------------------------------------------------------
(define (define-psi-stimulus gsn modulator-name)
"
  It associates a stimulus to an OpenPsi-modulator

  gsn:
    - A valid GroundedSchemaNode. Since there is no type checking done, be
      be sure that it is properly defined.

  modulator-name:
    - The name of the modultor that is stimulated by the execution of the
      function associated with the GroundedSchemaNode.
"
    (EvaluationLink
        (PredicateNode (string-append (psi-prefix-str) "Stimulus"))
        (ListLink
            (if (equal? (cog-type gsn) 'GroundedSchemaNode)
                gsn
                (error (string-append "pass GroundedSchemaNode as the 1st "
                     "argument in define-psi-stimulus"))
            )
            (ConceptNode (string-append (psi-prefix-str) modulator-name))
        )
    )
)

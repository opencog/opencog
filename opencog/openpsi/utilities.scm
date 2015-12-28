; Copyright (C) 2015 OpenCog Foundation
;
; Helper functions used by all sorts of psi scheme scripts
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-11-25
;

(use-modules (opencog rule-engine))

; Initialize seed of pseudo-random generator using current time
(let ( (time (gettimeofday) )
     )
    (set! *random-state*
         (seed->random-state
             (+ (car time) (cdr time) )
         )
    )
)

; --------------------------------------------------------------
; Fuzzy logic related functions
; --------------------------------------------------------------
; Return the probability of x equals to t (the target number)
; fuzzy_equal(x,t,a) = 1/(1+a(x-t)^2)
; a is the  parameter, the bigger a, the closer to crisp set
; After plotting via gnuplot for a while, it seems for t falls in [0,1], a=100
; is a good choice
(define (fuzzy_equal x t a)
    (/ 1
        (+ 1
            (* a (- x t) (- x t))
        )
    )
)

; Return the probability x falls in [min, max]
; a is the parameter, the bigger a, the closer to crisp set
; For x falls in [0,1], a=100 seems a good choice
(define (fuzzy_within x min_value max_value a)
    (if (< x min_value)
        (fuzzy_equal x min_value a)

        (if (> x max_value)
            (+ 0.999
               (* (random:uniform) 0.001)
            )

            (+ 0.99
               (* (random:uniform) 0.01)
            )
        ); if

    ); if
)

; Ruturn the probability x is smaller than t,
; a is the parameter, the bigger a, the closer to crisp set
(define (fuzzy_less_than x t a)
    (if (> x t)
        (fuzzy_equal x t a)
        1
    )
)

(define (fuzzy_low x t a)
    (fuzzy_less_than x t a)
)

; Ruturn the probability x is greater than t,
; a is the parameter, the bigger a, the closer to crisp set
(define (fuzzy_greater_than x t a)
    (if (< x t)
        (fuzzy_equal x t a)
        1
    )
)

(define (fuzzy_high x t a)
    (fuzzy_greater_than x t a)
)

; --------------------------------------------------------------
;
; Helper functions for numbers
;

; Return the normalized value ( falls in [0, 1] ) of x, given min and max values
(define (normalize x min_value max_value)
    (/ (- x min_value)
       (- max_value min_value)
    )
)

; Return the clipped value ( fallss in [#2, #3] ) of #1, given min (#2) and max (#3) values
(define (clip_within x min_value max_value)
    (cond
        ( (< x min_value)
          min_value
        )

        ( (> x max_value)
          max_value
        )

        (else x)
    )
)

; --------------------------------------------------------------

; Get pleasure based on previously/ currently selected demand goal
;
(define (get_pleasure_value)
    (let* (  (previous_demand_evaluation_link
                 (get_reference (ConceptNode "PreviousDemandGoal"))
             )
             (current_demand_evaluation_link
                 (get_reference (ConceptNode "CurrentDemandGoal"))
             )
             (previous_demand_satisfaction (random:uniform) ) ; initialize with random values
             (current_demand_satisfaction (random:uniform) )
             (energy (get_truth_value_mean (cog-tv EnergyDemandGoal)) )
             (integrity (get_truth_value_mean (cog-tv IntegrityDemandGoal)) )
          )

          ; set previous demand satisfaction (if available)
          (if (not (null? previous_demand_evaluation_link) )
              (set! previous_demand_satisfaction
                  (get_truth_value_mean (cog-tv previous_demand_evaluation_link))
              )
          )

          ; set current demand satisfaction (if available)
          (if (not (null? current_demand_evaluation_link) )
              (set! current_demand_satisfaction
                  (get_truth_value_mean (cog-tv current_demand_evaluation_link))
              )
          )

          ; return the pleasure depending on previous and current demand satisfactions
          (+ (* 0.25 current_demand_satisfaction)
             (* 0.15 previous_demand_satisfaction)
             (* 0.6 energy) ; we concern more on the energy demand, so use it to bias the plesaure value
;             (* 0.3 integrity)
          )
    )
)

; --------------------------------------------------------------
; Miscellaneous
; --------------------------------------------------------------
; Return a random member of the given list,
; return an empty list, if the given list is empty.
(define (random_select selection_list)
    (if (null? selection_list)
        (list)

        (list-ref selection_list
            (random (length selection_list) )
        )
    )
)

; Given a list of atoms l return the atom with the lowest TV strengh
(define (atom_with_lowest_tv_mean l)
  (min-element-by-key l (lambda (atom) (get_truth_value_mean (cog-tv atom))))
)

; Given a list of atoms l return the atom with the lowest TV strengh
(define (atom_with_highest_tv_mean l)
  (max-element-by-key l (lambda (atom) (get_truth_value_mean (cog-tv))))
)

; Given a list of atoms l return them sorted by TVs (ascending order)
(define (sort_by_tv l)
  (sort l (lambda (a1 a2) (>=  (get_truth_value_mean (cog-tv a1))
                               (get_truth_value_mean (cog-tv a2))
                               )
                  )
        )
)


; --------------------------------------------------------------
; Helper Functions
; --------------------------------------------------------------
(define (psi-clean-demand-gets set-link)
"
  Returns a list aftering removing the 'SetLink + 'ListLink + 'NumberNode
  from result of `cog-execute!` of 'GetLink wrapping psi-demand-pattern with or
  without additional clauses.

  set-link:
    - A 'SetLink that is a result of running `cog-execute!` of 'GetLink
      wrapping psi-demand-pattern.
"
; TODO: Make this generic for pattern gets
    (define (list-merge list-of-list) (fold append '() list-of-list))
    (define (is-nn? x) (equal? (cog-type x) 'NumberNode))
    (remove! is-nn?
        (list-merge (map cog-outgoing-set (cog-outgoing-set set-link))))
)

; --------------------------------------------------------------
(define (psi-prefix-str)
"
  Returns the string used as a prefix to all OpenPsi realted atom definition
"
    "OpenPsi: "
)

; --------------------------------------------------------------
(define (psi-suffix-str a-string)
"
  Returns the suffix of that follows `psi-prefix-str` sub-string.

  a-string:
    - a string that should have the`psi-prefix-str`

"
    (let ((z-match (string-match (psi-prefix-str) a-string)))
        (if z-match
            (match:suffix z-match)
            (error (string-append "The string argument must have the prefix: "
                "\"" (psi-prefix-str) "\"") )
        )
    )
)

; --------------------------------------------------------------
; Functions for OpenPsi active-schema-pool
; --------------------------------------------------------------
(define (psi-run)
"
  The main function that runs OpenPsi active-schema-pool. The active-schema-pool
  is a rulebase, that is modified depending on the demand-values, on every
  cogserver cycle.
"

    (cog-fc (SetLink)
        (ConceptNode (string-append (psi-prefix-str) "active-schema-pool"))
        (SetLink))
)

; --------------------------------------------------------------
(define (psi-select-goal gpn)
"
  Goal are defined as demands choosen for either increase or decrease in
  their demand values. The choice for being the current-goal is made by pattern
  matching over the demands by using the GroundedPredicateNode passed as a
  constraint.

  The GroundedPredicateNode passed should tag the demands for increase or
  decrease by evaluating the demands that satisfy the conditions set through
  its goal-selection function. The function should tag the goal representing
  atom (the demand-ConceptNode) for being increased or decreased.

  gpn:
    - GroundedPredicateNode that names an goal-selection function that will
      choose the demands . Its function should take a single demand-ConceptNode
      and return True-TruthValue `(stv 1 1)`  or False-TruthValue `(stv 0 1)`
      in addition to tagging the demand-ConceptNodes as either,

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Increase\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      (StateLink
          (Node (string-append (psi-prefix-str) \"action-on-demand\"))
          (ListLink
              (ConceptNode (string-append (psi-prefix-str) \"Decrease\"))
              (ConceptNode (string-append (psi-prefix-str) \"Energy\"))))

      The tags are necessary because, that is the means for signaling what type
      of actions should be taken, in effect it is the demand-goal. For example,
      if the action-on-demand is Increase, then only the actions of type
      Increase would be choosen.

"

    ; XXX: Should there be weight b/n the different demand-goals? For now a
    ; a random choice of demands is assumed. In subsequent steps. The
    ; demand-value could possibly be used for that.
    (define (get-demand) (psi-clean-demand-gets (cog-execute!
        (GetLink
            (VariableList
                (assoc-ref (psi-demand-pattern) "var"))
            (AndLink
                (assoc-ref (psi-demand-pattern) "pat")
                (EvaluationLink
                    gpn
                    (ListLink (VariableNode "Demand"))))))))

    ; check arguments
    (if (not (equal? (cog-type gpn) 'GroundedPredicateNode))
        (error "Expected DefinedPredicateNode got: " gpn))

    ; TODO: 1. deal with multiple returns
    ;       2. check if the demadns have ben correctly tagged  instead add psi-register-goal-selector
    (get-demand)
)

; --------------------------------------------------------------
(define (psi-get-current-goal)
"
  Returns the demand-ConceptNode that has been choosen for action presently.
"
   (define (get-psi-goal) (cog-execute!
      (GetLink
        (TypedVariableLink
            (VariableNode "demand")
            (TypeNode "ConceptNode"))
        (AndLink
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ChoiceLink
                    (ListLink
                        (ConceptNode
                            (string-append (psi-prefix-str) "Decrease"))
                        (VariableNode "demand"))
                    (ListLink
                        (ConceptNode
                            (string-append (psi-prefix-str) "Increase"))
                        (VariableNode "demand"))))
            (EvaluationLink ; Act only if their is such a demand.
                (GroundedPredicateNode "scm: psi-demand?")
                (ListLink
                    (VariableNode "demand")))))
    ))

    (let* ((set-link (get-psi-goal))
          (result (car (cog-outgoing-set set-link)))
          )
          (cog-delete set-link)
          result
    )
)

; --------------------------------------------------------------
(define (psi-get-current-action-type)
"
  This returns a node of the type of effect that the current-goal have.
"
   (define (get-psi-action-type) (cog-execute!
        (GetLink
            (TypedVariableLink
                (VariableNode "effect-type")
                (TypeNode "ConceptNode"))
            (StateLink
                (Node (string-append (psi-prefix-str) "action-on-demand"))
                (ListLink
                    (VariableNode "effect-type")
                    (psi-get-current-goal))))
    ))

    (let* ((set-link (get-psi-action-type))
          (result (car (cog-outgoing-set set-link)))
          )
          (cog-delete set-link)
          result
    )
)

; --------------------------------------------------------------
(define (psi-update-asp demand-node gpn)
"
  It modifies the member action-rules of the active-schema-pool.
"
;not sure if should be part of psi-select-actions, why separate them?
    ;(let ((choosen-actions (psi-select-actions demand-node gpn)))
    ;    ()
    ;)

(display "WIP")
)

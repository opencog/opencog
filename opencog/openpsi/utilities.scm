; Copyright (C) 2015 OpenCog Foundation
;
; Helper functions used by all sorts of psi scheme scripts
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-11-25
;

(use-modules (ice-9 regex)) ; For string-match
(use-modules (srfi srfi-1)) ; For set-difference

(use-modules (opencog) (opencog rule-engine))

; --------------------------------------------------------------
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
             (energy (tv-mean (cog-tv EnergyDemandGoal)) )
             (integrity (tv-mean (cog-tv IntegrityDemandGoal)) )
          )

          ; set previous demand satisfaction (if available)
          (if (not (null? previous_demand_evaluation_link) )
              (set! previous_demand_satisfaction
                  (tv-mean (cog-tv previous_demand_evaluation_link))
              )
          )

          ; set current demand satisfaction (if available)
          (if (not (null? current_demand_evaluation_link) )
              (set! current_demand_satisfaction
                  (tv-mean (cog-tv current_demand_evaluation_link))
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
(define (random-select selection-list)
    (if (null? selection-list)
        (list)

        (list-ref selection-list
            (random (length selection-list) )
        )
    )
)

; Given a list of atoms l return the atom with the lowest TV strengh
(define (atom_with_lowest_tv_mean l)
  (min-element-by-key l (lambda (atom) (tv-mean (cog-tv atom))))
)

; Given a list of atoms l return the atom with the lowest TV strengh
(define (atom_with_highest_tv_mean l)
  (max-element-by-key l (lambda (atom) (tv-mean (cog-tv))))
)

; Given a list of atoms l return them sorted by TVs (ascending order)
(define (sort_by_tv l)
  (sort l (lambda (a1 a2) (>=  (tv-mean (cog-tv a1))
                               (tv-mean (cog-tv a2))
                               )
                  )
        )
)

; --------------------------------------------------------------
; Helper Functions
; --------------------------------------------------------------
(define (list-merge list-of-list)
"
  A helper function for merging list of lists into a single list. Specially for
  results of `par-map`. For single threaded map use `append-map`.
"
    (fold append '() list-of-list)
)

; --------------------------------------------------------------
(define (cog-tv-mean> mean1 mean2)
    (if (> mean1 mean2)
        (stv 1 1)
        (stv 1 0)
    )
)

(define (cog-tv-mean< mean1 mean2)
    (if (< mean1 mean2)
        (stv 1 1)
        (stv 1 0)
    )
)

; --------------------------------------------------------------
(define-public (psi-prefix-str)
"
  Returns the string used as a prefix to all OpenPsi realted atom definition
"
    "OpenPsi: "
)

; --------------------------------------------------------------
(define-public (psi-suffix-str a-string)
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

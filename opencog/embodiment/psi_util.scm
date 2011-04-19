;
; Helper functions used by all sorts of psi scheme scripts
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-04-18
;

; Return the probability of x equals to t (the target number)
; fuzzy_equal(x,t,a) = 1/(1+a(x-t)^2)
; a is the  parameter, the bigger a, the closer to crisp set
; After plotting via gnuplot for a while, it seems for t falls in [0,1], a=100 is a good choice 
(define (fuzzy_equal x t a)
    (/ 1
        (+ 1
            (* a (- x t) (- x t)) 
        )  
    )     
)

; Ruturn the probability x falls in [min, max] 
; a is the parameter, the bigger a, the closer to crisp set
; For x falls in [0,1], a=100 seems a good choice
(define (fuzzy_within x min_value max_value a)
    (if (< x min_value)
        fuzzy_equal(x min_value a) 
        fuzzy_equal(x max_value a)

    ) 
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

; TODO: Finish this function
(define (get_modulator_value modulator_name)
    (random:uniform)
)

; Get pleasure based on previously/ currently selected demand goal
; TODO: Finish this function
; get_pleasure(0) := +( *(0.35 get_current_demand_goal_truth_value)
;                      *(0.65 get_previous_demand_goal_truth_value)
;
(define (get_pleasure_value)
    (random:uniform) 
)


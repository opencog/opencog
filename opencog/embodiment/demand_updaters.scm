;
; Demand updaters
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-12-09
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

;(define (AffiliationDemandUpdater)
;    (let* ( (proximity
;                 (get_proximity (get_owner) (get_self) )
;            )
;
;            (fitness
;                (get_latest_predicate_truth_value_mean "fitness") 
;            )
;          )
;
;         ; Note: We take fitness into consideration, because when you are in bad
;         ; situation, you would probably miss your family and friends
;          (* (expt proximity 0.5) 
;             (expt fitness 0.5)  
;          )   
;    )
;)

; Connect affiliation demand with dialog system  
(define (AffiliationDemandUpdater)
    (let ( (eval_has_unanswered_question
               (EvaluationLink (PredicateNode "has_unanswered_question") )
           )
           (eval_has_dramatic_changes
               (EvaluationLink (PredicateNode "has_dramatic_changes") )
           )
         ) 

         (- (+  0.75 
                (* (random:uniform) 0.25)
            )

            (+ 
                (* 0.4  (get_truth_value_mean (cog-tv eval_has_unanswered_question) ) )
                (* 0.35 (get_truth_value_mean (cog-tv eval_has_dramatic_changes) ) )
            )    
         ); -
    )
)

; If the world changes more frequently, more certainty demand is required 
(define (CertaintyDemandUpdater)
    (let ( (changes_with_tv (get_changes_with_tv) )
           (changes_with_arg (get_changes_with_arg) )
           (change_with_tv_level 0)
           (change_with_arg_level 0)
         )
         ; Normalize levels of changes to [0, 1]
         ;
         ; Note: The formular is 
         ;       xxx_level = 1 / (1 + a*x^2)
         ;
         ; TODO: we add a random number to introduce some noise, which would
         ;       make this function still works even the virtual world is not 
         ;       ready to populate interesting stuff. Remove these random noise later. 
         ;
         
         (set! change_with_tv_level
             (/ 1
                (+ 1 
                   (* 0.001 
                      (+ (length changes_with_tv) (* 20 (random:uniform)) )
                      (+ (length changes_with_tv) (* 20 (random:uniform)) )
                   )  
                )
             )
         )

         (set! change_with_arg_level
             (/ 1
                (+ 1 
                   (* 0.001
                      (+ (length changes_with_arg) (* 1 (random:uniform)) )
                      (+ (length changes_with_arg) (* 1 (random:uniform)) )
                   )  
                )
             )
         )

         ; Return certainty level
         (+ (* 0.45 change_with_tv_level) 
            (* 0.35 change_with_arg_level)
            (* 0.20 (get_truth_value_mean (cog-tv CertaintyDemandGoal)) )
         )
    ) 
)

; TODO: Think about how to connect certainty demand with dialog system? 
;       A possible question might be evaluate how much infomaton the agent knows
;       about the world, objects and agents. And then it can ask questions on that stuff.
;(define (CertaintyDemandUpdater)
;    (let* ( (curious_about_evaluation_link_list
;                (query_atom_space (find_evaluation_link "curious_about") ) 
;            )
;
;            (familiar_with_evaluation_link_list 
;                (query_atom_space (find_evaluation_link "familiar_with") )
;            )   
;
;            (know_evaluation_link_list
;                (query_atom_space (find_evaluation_link "know") ) 
;            )
;
;            (curious_about_level 0)
;            (familiar_with_level 0)
;            (know_level 0)
;          )
;
;          ; Accumulate all the truth value of curious_about EvaluationLink
;          (map 
;              (lambda (curious_about_evaluation_link)
;                  (set! curious_about_level 
;                      (+ curious_about_level 
;                          (get_truth_value_mean (cog-tv curious_about_evaluation_link) ) 
;                      ) 
;                  ) 
;              )
;
;              curious_about_evaluation_link_list
;          ); map
;
;          ; Accumulate all the truth value of familiar_with EvaluationLink
;          (map 
;              (lambda (familiar_with_evaluation_link)
;                  (set! familiar_with_level 
;                      (+ familiar_with_level 
;                          (get_truth_value_mean (cog-tv familiar_with_evaluation_link) ) 
;                      ) 
;                  ) 
;              )
;
;              familiar_with_evaluation_link_list
;          ); map
;
;          ; Accumulate all the truth value of know EvaluationLink
;          (map 
;              (lambda (know_evaluation_link)
;                  (set! know_level 
;                      (+ know_level 
;                          (get_truth_value_mean (cog-tv know_evaluation_link) ) 
;                      ) 
;                  ) 
;              )
;
;              know_evaluation_link_list
;          ); map
;
;          ; Normalize levels of familiar_with, know and curious_about to [0, 1]
;          ;
;          ; Note: The formular is 
;          ;       xxx_level = 1 / {1 + a * exp[-(xxx_level+rand)/b] }
;          ;
;          ;       a controls the output xxx_level when input xxx_level equals 0, 
;          ;       the bigger a, the smaller the output
;          ;
;          ;       b controls the corresponding input xxx_level when output 
;          ;       xxx_level approaching 1, bigger b would result larger input 
;          ;       xxx_level to get output approaching 1. 
;          ;       
;          ;       After playing with gnuplot for a while, I found (a=100, b=1.5)
;          ;       seems reasonable, which would make the output xxx_level close 
;          ;       to 0 when input is 0, and make the output xxx_level approaching
;          ;       1, when input is 20. 
;          ;
;          ; TODO: we add a random number to introduce some noise, which would
;          ;       make this function still works even the virtual world is not 
;          ;       ready to populate interesting stuff. Remove these random noise later. 
;          ;
;          
;          (set! curious_about_level
;              (/ 1
;                 (+ 1 
;                    (* 100 
;                       (exp (/ (* (+ curious_about_level (* 15 (random:uniform))) -1)
;                               1.5
;                            )
;                       ) 
;                    )  
;                 )
;              )
;          )
;
;          (set! familiar_with_level
;              (/ 1
;                 (+ 1 
;                    (* 100 
;                       (exp (/ (* (+ familiar_with_level (* 15 (random:uniform))) -1)
;                               1.5
;                            )
;                       ) 
;                    )  
;                 )
;              )
;          )
;
;          (set! know_level
;              (/ 1
;                 (+ 1 
;                    (* 100 
;                       (exp (/ (* (+ know_level (* 15 (random:uniform))) -1)
;                               1.5
;                            )
;                       ) 
;                    )  
;                 )
;              )
;          )
;
;          ; Return certainty level
;          ;
;          ; note: At first, you are curious about something, after you playing
;          ;       with it for a while, you are familiar with it, at last you 
;          ;       claim you know it.
;          ;
;          (+ (* 0.2 curious_about_level) 
;             (* 0.3 familiar_with_level)
;             (* 0.5 know_level)
;          )
;
;    ); let*
;); define 

; PAI::setActionPlanStatus is responsible for creating actionDone/ actionFailed predicates
; 
; Plot the graph in gnuplot
; set contour both; set grid; set xlabel "Action Done";set ylabel "Action Fail"; set zlabel "Competence";
; set xrange [0:100]; set yrange [0:100]; set isosample 50, 50; splot x/(x+y**1.5)
;
(define (CompetenceDemandUpdater)
    (let* ( (plan_done_at_time_link_list 
                (query_atom_space (find_at_time_link "actionDone") ) 
            )

            (plan_failed_at_time_link_list
                (query_atom_space (find_at_time_link "actionFailed") ) 
            )

            (plan_done_number (length plan_done_at_time_link_list) )
            (plan_failed_number (length plan_failed_at_time_link_list) )
          )
 
          ; avoid division by zero and decimal part will force guile return floating
          ; numbers while division. That is return 0.33333, rather than 1/3, 
          ; which will confuse the caller via cpp code. 
          (set! plan_done_number
              (+ plan_done_number 
                 (* (random:uniform) 2) 
                 3.0123 
              )
          )

          (set! plan_failed_number
              (+ plan_failed_number 
                 (* (random:uniform) 1) 
                 0.0123
              ) 
          )

          (/ plan_done_number
             (+ plan_done_number (expt plan_failed_number 1.5) )
          )
;
;          (/ plan_done_number
;             (+ plan_done_number (* 4 plan_failed_number) )
;          )
    ); let*
)


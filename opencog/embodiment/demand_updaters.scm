;
; Demand updaters
;
; @author Zhenhua Cai czhedu@gmail.com
; @date   2011-05-17
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
    (let* ( (proximity
                 (get_proximity (get_owner) (get_self) )
            )

            (fitness
                (get_latest_predicate_truth_value_mean "fitness") 
            )
          )

         ; Note: We take fitness into consideration, because when you are in bad
         ; situation, you would probably miss your family and friends
          (* (expt proximity 0.5) 
             (expt fitness 0.5)  
          )   
    )
)

(define (CertaintyDemandUpdater)
    (let* ( (curious_about_evaluation_link_list
                (query_atom_space (find_evaluation_link "curious_about") ) 
            )

            (familiar_with_evaluation_link_list 
                (query_atom_space (find_evaluation_link "familiar_with") )
            )   

            (know_evaluation_link_list
                (query_atom_space (find_evaluation_link "know") ) 
            )

            (curious_about_level 0)
            (familiar_with_level 0)
            (know_level 0)
          )

          ; Accumulate all the truth value of curious_about EvaluationLink
          (map 
              (lambda (curious_about_evaluation_link)
                  (set! curious_about_level 
                      (+ curious_about_level 
                          (get_truth_value_mean (cog-tv curious_about_evaluation_link) ) 
                      ) 
                  ) 
              )

              curious_about_evaluation_link_list
          ); map

          ; Accumulate all the truth value of familiar_with EvaluationLink
          (map 
              (lambda (familiar_with_evaluation_link)
                  (set! familiar_with_level 
                      (+ familiar_with_level 
                          (get_truth_value_mean (cog-tv familiar_with_evaluation_link) ) 
                      ) 
                  ) 
              )

              familiar_with_evaluation_link_list
          ); map

          ; Accumulate all the truth value of know EvaluationLink
          (map 
              (lambda (know_evaluation_link)
                  (set! know_level 
                      (+ know_level 
                          (get_truth_value_mean (cog-tv know_evaluation_link) ) 
                      ) 
                  ) 
              )

              know_evaluation_link_list
          ); map

          ; Normalize levels of familiar_with, know and curious_about to [0, 1]
          ;
          ; Note: The formular is 
          ;       xxx_level = 1 / {1 + a * exp[-(xxx_level+rand)/b] }
          ;
          ;       a controls the output xxx_level when input xxx_level equals 0, 
          ;       the bigger a, the smaller the output
          ;
          ;       b controls the corresponding input xxx_level when output 
          ;       xxx_level approaching 1, bigger b would result larger input 
          ;       xxx_level to get output approaching 1. 
          ;       
          ;       After playing with gnuplot for a while, I found (a=100, b=1.5)
          ;       seems reasonable, which would make the output xxx_level close 
          ;       to 0 when input is 0, and make the output xxx_level approaching
          ;       1, when input is 20. 
          ;
          ; TODO: we add a random number to introduce some noise, which would
          ;       make this function still works even the virtual world is not 
          ;       ready to populate interesting stuff. Remove these random noise later. 
          ;
          
          (set! curious_about_level
              (/ 1
                 (+ 1 
                    (* 100 
                       (exp (/ (* (+ curious_about_level (* 15 (random:uniform))) -1)
                               1.5
                            )
                       ) 
                    )  
                 )
              )
          )

          (set! familiar_with_level
              (/ 1
                 (+ 1 
                    (* 100 
                       (exp (/ (* (+ familiar_with_level (* 15 (random:uniform))) -1)
                               1.5
                            )
                       ) 
                    )  
                 )
              )
          )

          (set! know_level
              (/ 1
                 (+ 1 
                    (* 100 
                       (exp (/ (* (+ know_level (* 15 (random:uniform))) -1)
                               1.5
                            )
                       ) 
                    )  
                 )
              )
          )

          ; Return certainty level
          ;
          ; note: At first, you are curious about something, after you playing
          ;       with it for a while, you are familiar with it, at last you 
          ;       claim you know it.
          ;
          (+ (* 0.2 curious_about_level) 
             (* 0.3 familiar_with_level)
             (* 0.5 know_level)
          )

    ); let*
); define 

; PAI::setActionPlanStatus is responsible for creating actionDone/ actionFailed predicates
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
              (+ plan_done_number 1.0123)
          )

          (set! plan_failed_number
              (+ plan_failed_number 1.0123) 
          )

          (/ plan_done_number
             (+ plan_done_number (* 5 plan_failed_number) )
          )
    )     
)

; TODO: TestEnergy is only used for debugging. Remove it once finished. 
(define (TestEnergyDemandUpdater)
    (random:uniform) 
)


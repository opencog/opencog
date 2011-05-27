;
; Action planner
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-05-27
;

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

; Return precondition/ goal given a psi rule
(define (get_psi_precondition rule)
    (list-ref (cog-outgoing-set rule) 0)
)

(define (get_psi_goal rule)
    (list-ref (cog-outgoing-set rule) 1) 
)

; Return #t if given atom is a Link    
(define (cog-link? atom)
    (cog-subtype? (cog-type atom) 'Link) 
)    

; Return #t if given atom is an Node
(define (cog-node? atom)
    (cog-subtype? (cog-type atom) 'Node) 
)

; Return a BinkLink used by pattern matcher to search psi rules given goal
(define (find_psi_rule goal)
    (BinkLink
        (VariableNode "$var_any")

        (ImplicationLink
            ; Pattern to be searched
            (ImplicationLink
                (VariableNode "$var_any")
                goal
            )

            ; Return value
            (ImplicationLink
                (VariableNode "$var_any")
                goal
            ) 
        )

    ); BinkLink 
)

; Return a BinkLink used by pattern matcher to search the action with suitable 
; groundings of all the variables that make the context True
;
; @param variables Usually it is the first outgoing of ForAllLink. 
;                  It may be a single VariableNode, a single TypedVariableLink
;                  or a ListLink of VariableNodes or TypedVariableLink. 
;  
; @note There's a chance that the action is empty. Especially in rules about 
;       relations, such as SomeOneKickYou => YouGetAngry  
;
(define (find_psi_action variables context action)
    (BindLink
        variables

        (ImplicationLink
            context

            (ListLink
                context
                action
            )
        )
    )
)

; Split an atom into context and action,
; the return value is 
;
; (list
;     (list context_atom) 
;     (list action_atom)
; )
;
(define (split_context_action atom)
    (if (cog-atom? atom)
        ; If the input is an atom   
        (begin
            (let ( (atom_type (cog-type atom) )
                 )

                 (cond 
                     ; If the input is an ExecutionLink, return the input as action
                     ( (equal? atom_type 'ExecutionLink)
                       (list (list) (list atom) )
                     )

                     ; If the input is a Node but NOT a VariableNode, return the input as context 
                     ( (cog-subtype? atom_type 'Node)
                         (list (list atom) (list) )
                     )

                     ; If the input is a Link, process its outgoings recursively
                     (else

                       (let  ( (atom_outgoings (cog-outgoing-set atom) )
                               (context_outgoings (list) )
                               (action_outgoings (list) )
                             ) 
                            
                             ; Split the outgoings into context and action
                             (map-in-order
                                 (lambda (outgoing)
                                     
                                     (let ( (split_result (split_context_action outgoing) )
                                          )

                                          (append context_outgoings (list-ref split_result 0) )
                                          (append action_outgoings (list-ref split_result 1) )
                                     ) 
                                      
                                 ); lambda
  
                                 atom_outgoings
  
                             ); map-in-order
  
                             ; Create and return the context and action
                             (list

                                 ; Build context atom
                                 (if (null? context_outgoings)
                                     (list)

                                     (list
                                         (apply cog-new-link
                                             (append (list atom_type)
                                                      context_outgoings  
                                                     (list (cog-tv atom) (cog-av-atom) )
                                             ) 
                                         )
                                     ); list
                                 ); if
  
                                 ; Build action atom
                                 (if (null? action_outgoings)
                                     (list) 

                                     (list 
                                         (apply cog-new-link
                                                (append (list atom_type)
                                                        action_outgoings
                                                        (list (cog-tv atom) (cog-av atom) )
                                                )  
                                         )
                                     ); list
                                 ); if
                             ); list
      
                       ); let

                     ); else

                 ); cond

            ); let
        ); begin

        ; If the input is NOT an atom, return empty list
        (list (list) (list) )

    ); if
); define    

; Calculate the truth value of the given atom
; It simply use the truth value the atom and its outgoings already holds.
; It will NOT try to ground the variables. So if there are variables in the atom 
; or its outgoings recursively, using cog-bind-crisp to ground the variables
(define (cal_truth_value atom)
    (let* ( (atom_type (cog-type atom) )
            (atom_outgoings (cog-outgoing-set atom) )
          )
 
          (cond
              ; If the outgoings is empty, return the truth value 
              ; of the current atom
              ( (null? atom_outgoings)
                (get_truth_value_mean (cog-tv atom) )
              ) 
 
              ; If it is an AndLink, the truth value is the 
              ; average of all the truth values of its outgoings
              ( (equal? atom_type 'AndLink)
 
                (let ( (outgoing_truth_values 
                           (map cal_truth_value atom_outgoings) 
                       )
                     )
                     
                     (/ (apply + outgoing_truth_values)
                        (length outgoing_truth_values) 
                     )   
                )
              )
 
              ; If it is an OrLink, the truth value is the maximum 
              ; value of all the truth values of its outgoings
              ( (equal? atom_type 'OrLink)
 
                (max
                    (map cal_truth_value atom_outgoings) 
                )
              )
 
              ; If it is an NotLink, the truth value is 1 minus 
              ; the maximum value of all the truth values of its 
              ; outgoings
              ( (equal? atom_type 'NotLink)
 
                (- 1
                   (max
                       (map cal_truth_value atom_outgoings) 
                   )  
                )
              )
 
              ; If it is of other types, simply return the truth value
              (else
                (get_truth_value_mean (cog-tv atom) )
              ) 
 
          ); cond
 
    );let* 

); define

; Check if the context of the given psi rule is true. 
;
; @param precondition Usually the first outgoing of an psi rule, i.e. ImplicationLink
;
; @return A list as follows, 
;
;         (list
;             (list true_context_1 action_1)
;             (list true_context_2 action_2)
;             ...
;         )    
;
; @note If the context is true, then all the variables in returned context and
;       action would be properly grounded. 
;
(define (is_psi_context_true rule)
    (let* ( (precondition 
                (list-ref (cog-outgoing-set rule) 0)
            )
            (context_atom_list (split_context_action precondition) )
            (context (list-ref context_atom_list 0) )
            (action (list-ref context_atom_list 1) )
            (variables (list) )
          ) 

          ; Get the variables of the rule,
          ; the first step is to get a ForAllLink, ExistsLink or AverageLink 
          ; contaning the rule, and then return the first outgoing of the link 
          (let search_variables ( (incomings (cog-incoming-set rule) )
                                ) 
               (if (not (null? incomings) )
                   (let* ( (first_incoming (car incomings) )
                           (first_incoming_type (cog-type first_incoming) )
                         )
                        
                         (if (or (equal? first_incoming_type 'ForAllLink)
                                 (equal? first_incoming_type 'ExistsLink)
                                 (equal? first_incoming_type 'AverageLink)
                             )

                             (set! variables 
                                   (list-ref (cog-outgoing-set first_incoming) 0)
                             )

                             (search_variables (cdr incomings) )
                         ); if

                   ); let* 
               ); let
          );let 

          (if (null? variables)

              ; If there's no variable in the psi rule, calculate the truth value
              ; of context directly
              (if (> (cal_truth_value context) 
                      0.5
                  )
                  (list (list context action) )
                  (list)
              )

              ; If there are variables in the psi rule, use pattern matcher to
              ; find suitable groundings that makes the context True
              (map unpack_query_result 
                  (query_atom_space (find_psi_action variables context action) )
              )   
              
          ); if
         
    ); let*
); define

; Make a plan for a given goal
;
; @return a list of (context action) and corresponding rules that would lead to
;         the given goal, like
;
;         (list 
;             (list
;                 solved_rule_1
;                 solved_rule_2
;                 ...
;             )
;
;             (list
;                 (list context_1 action_1)
;                 (list context_2 action_2)
;                 ...
;             )   
;         )
;
(define (make_psi_plan goal)
    (let ( (planned_goal_list (list) )
           (reachable_rule_list (list) )
           (unreachable_rule_list (list) )
           (context_action_list (list) )
           (solved_rule_list (list) )
         )

         (let do_plan ( (current_goal goal)
                      )

             ; If current goal is not empty and it has not been planned yet,
             ; make a plan for it
             (if (and (not (null? current_goal) 
                            (equal? (member current_goal planned_goal_list) #f)
                      )    
                 )    

                ; Get all the rules related to current goal 
                (let ( (available_rule_list
                           (query_atom_space (find_psi_rule current_goal) ) 
                       )
                     ) 

                    ; Append current goal to end of the planned goal list,
                    ; then it will not be planned twice. 
                    (set! planned_goal_list 
                          (append planned_goal_list (list current_goal) )
                    )

                    (if (null? available_rule_list)
                        ; If there's no rule directly attach to current goal, 
                        ; we should dive into its outgoings             
                        (let ( (atom_type (cog-type current_goal) )
                               (atom_outgoings (cog-outgoing-set current_goal) )
                             )
  
                             (if (not (null? atom_outgoings) )
                                 (cond 
                                     ( (equal? atom_type 'AndLink)
                                       (map do_plan atom_outgoings)
                                     ) 
  
                                     ( (equal? atom_type 'OrLink)
                                       (do_plan
                                           (random_select atom_outgoings) 
                                       )
                                     )    
                                 ); cond
                             ); if
                        ); let
                       
                        ; If there are rules attaching to the current goal
                        (let ( (selected_rule (list) )
                               (search_result (list) )
                             )
  
                             ; Search a rule with True context 
                             (let search_rule_with_true_context ( (rule_list available_rule_list)
                                                                )

                                  (if ( not (null? rule_list) )
                                      (let ( (current_rule (car rule_list) )
                                           )

                                           (if (not (equal? (member current_rule unreachable_rule_list)
                                                             #f
                                                    )
                                               )    
                                               
                                               ; Search
                                               (set! search_result 
                                                     (is_psi_context_true (car rule_list) )
                                               ) 
                        
                                               (if (null? search_result)
                                                   ; Not found
                                                   (begin
                                                       (set! unreachable_rule_list 
                                                             (append unreachable_rule_list (list current_rule) ) 
                                                       )
                                                       (search_rule_with_true_context (cdr rule_list) ) 
                                                   )   

                                                   ; Found
                                                   (begin
                                                       (set! reachable_rule_list
                                                             (append reachable_rule_list (list current_rule) )
                                                       )
                                                       (set! selected_rule (car rule_list) )
                                                   )   
                                               ); if 

                                           ); if 

                                      ); let
                                  ); if
                             ); let
  
                             (if (null? selected_rule)
                                 ; If there's no rule with True context, randomly select a rule
                                 ;
                                 ; TODO: we should take advantage of weights (truth value) of
                                 ;       rules for selection using roulette wheel selection. 
                                 (begin
                                     (set! selected_rule 
                                           (random_select available_rule_list)
                                     )
  
                                     (do_plan
                                         (get_psi_precondition selected_rule) 
                                     )
                                 )
   
                                 ; If found a rule with True context
                                 (let (selected_context_action 
                                          (random_select search_result)
                                      ) 

                                      (if (equal? (member selected_context_action context_action_list)
                                                  #f
                                          )

                                          (set! context_action_list 
                                                (append context_action_list
                                                        (list selected_context_action)
                                                )
                                          )

                                          (set! solved_rule_list
                                                (append solved_rule_list
                                                        (list selected_rule)
                                                ) 
                                          )
                                      )
                                 ); let
   
                             ); if
  
                        ); let
                    ); if (null? available_rule_list)

                ); let
             ); if

         ); let do_plan
    ); let    
); define


;
; @file embodiment/rules_core.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2010-12-09
;
; Scheme core functions for adding Modulators, Demands and Rules etc. into AtomSpace
;
; TODO:
;
; 1. Remove WILD_CARD_STR '_*_', use ForAllLink instead
; 2. Remove get_handle_from_list, use native Atom retrieving functions instead
; 3. Use CompositeTruthValue for Rule, currently SimpleTruthValue is used
;

;******************************************************************************
;
; How does OpenPsi components represented in AtomSpace
;
;==============================================================================
;
; Modulator is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "modulator_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "modulator_schema_name"
;         ListLink 
;             PET_HANDLE

;==============================================================================
;
; DemandSchema/DemandValue is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "demand_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "demand_schema_name"
;         ListLink
;             PET_HANDLE
;
; DemandValue is the output of DemandSchema.
;
;==============================================================================
;
; DemandGoal is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode: "demand_name_goal" 
;                        (SimpleTruthValue indicates how well the demand is satisfied)
;                        (ShortTermInportance indicates the urgency of the demand)
;     EvaluationLink
;         GroundedPredicateNode: FuzzyWithin
;         ListLink
;             NumberNode: min_acceptable_value
;             NumberNode: max_acceptable_value
;             SimilarityLink (stv 1.0 1.0)
;                 NumberNode: "demand_value"
;                 ExecutionOutputLink
;                     GroundedSchemaNode: "demand_schema_name"
;                     ListLink
;                         PET_HANDLE
;
;==============================================================================
;
; Finally, Rule is represented as:
;
; PredictiveImplicationLink (CompositeTruthValue indicates the mode strengths)
;     AndLink
;         AtTimeLink
;             TimeNode CURRENT_TIMESTAMP 
;             EvaluationLink
;                 GroundedPredicateNode "precondition_name"
;                 ListLink (empty)
;         ExecutionLink
;             GroundedSchemaNode "schema_name"
;             ListLink
;                 Node:params ...
;                 ...
;     AtTimeLink
;         TimeNode CURRENT_TIMESTAMP
;         EvaluationLink
;             PredicateNode: "demand_name_goal" 
;                            (SimpleTruthValue indicates how well the demand is satisfied)
;                            (ShortTermInportance indicates the urgency of the demand)
;
;==============================================================================
;
; Reference:
;
;     http://wiki.opencog.org/w/OpenCogPrime:FunctionNotation
;     http://wiki.opencog.org/w/TimeServer
;
;******************************************************************************

;==============================================================================
;
; Information type used by print_debug_info function
;
(define INFO_TYPE_SUCCESS "SUCCESS")
(define INFO_TYPE_FAIL    "FAIL")
(define INFO_TYPE_WARN    "WARN")

;==============================================================================
;
; Print some information for degugging, if you dislike them, just comment the body of the function
;
(define (print_debug_info info_type function_name info)
    (display (string-append info_type " in " function_name ": " info)
    )
    (newline)
)

;==============================================================================
;
; Default Attention Values
;
; Settings here are the same with "AttentionValue.h" line 48-50
;
(define DEFAULT_STI 0)
(define DEFAULT_LTI 0)
(define DEFAULT_VLTI #f)

;==============================================================================
;
; These Default values are only used for loading Demand, Rules etc, and 
; has nothing to do with other C++ code
;
; Default Attention Value 
(define (DEFAULT_AV) 
    (cog-new-av DEFAULT_STI 1 #t)
)

; Default Simple Truth Value
(define (DEFAULT_STV) 
    (cog-new-stv 0 0) 
)

;==============================================================================
;
; Handles for pet and its owner, 
; which is used by add_action, if its action parameters contain 'self', or 'owner'
;
; Since Scheme shell would never know these Handles automatically,
; you should call the scheme scripts below, 
; 
;     (set! PET_HANDLE (get_agent_handle "agent_id") )
;     (set! OWNER_HANDLE (get_owner_handle "owner_id") )
;
; firstly, in C++ code before actually loading any Rules!
;
(define PET_HANDLE (list) )
(define OWNER_HANDLE (list) )

;==============================================================================
;
; Current timestamp
;
; It would be used by ATTimeLink and TimeNode, since Scheme shell would never know 
; unless you tell it in C++ code, as follows 
;
;     (set! CURRENT_TIMESTAMP "438")
; 
; How to interpret the timestamp depends on the epoch and unit of time,
;
; Epoch in OpenCog is defined by PAIUtils::epoch at "PAIUtils.cc" line 35.
; In regard to time unit, it would be 0.1 seconds or 0.01 seconds, depending  on the
; macro definition DATETIME_DECIMAL_RESOLUTION (PAIUtils::getTimeInfo "PAIUtils.cc" line 141-149)
;
; Learn more
;     http://wiki.opencog.org/w/TimeServer
;

(define CURRENT_TIMESTAMP (list) )

;==============================================================================

; Association lists for retrieving the Handle of OpenPsi component given its name
;
(define modulator_list (list) )
(define demand_schema_list (list) )
(define demand_goal_list (list) )

; Get the Handle from specific handle_assoc_list given key_name, return null once fails
;
(define (get_handle_from_list handle_assoc_list key_name)
    (let ( (handle_returned (assoc-ref handle_assoc_list key_name) )
         )
         (if (not handle_returned)
             (begin
                 (set! handle_returned (list) )
                 (print_debug_info INFO_TYPE_FAIL "get_handle_from_list" 
                                   (string-append "Can not find " key_name)
                 )  
             )
         );if
         handle_returned
    );let
);define

;==============================================================================
;
; Get the agent Handle given agent_id
;
; The function is a Scheme version of AtomSpaceUtil::getAgentHandle ("AtomSpaceUtil.cc") \n
; \n
; Agent here has nothing to do with Mind Agent. \n
; Agent here can be a virtual pet, an avatar in video games or humonoid robot. \n
; Don't be confused with Mind Agent! \n
;
(define (get_agent_handle agent_id)
    (let ( (agent_handle (cog-node 'PetNode agent_id) )
         )

         (if (null? agent_handle)
             (begin
                 (set! agent_handle (cog-node 'AvatarNode agent_id) )

                 (if (null? agent_handle)
                     (begin
                         (set! agent_handle (cog-node 'HumanoidNode agent_id) )

                         (if (null? agent_handle)
                             (print_debug_info INFO_TYPE_FAIL "get_agent_handle"
                                               (string-append "Can not find any agent node named " 
                                                               agent_id
                                               )
                             )

                             (print_debug_info INFO_TYPE_SUCCESS "get_agent_handle"
                                               (string-append "Found an HumanoidNode named " agent_id)
                             )
                         );if
                     );begin

                     (print_debug_info INFO_TYPE_SUCCESS "get_agent_handle"
                                       (string-append "Found a AvatarNode named " agent_id)
                     )
                 );if
             );begin

             (print_debug_info INFO_TYPE_SUCCESS "get_agent_handle" 
                               (string-append "Found a PetNode named " agent_id)
             )
         );if
        agent_handle ; return the Handle found (note: it might be null)
    );let
);define

;==============================================================================
;
; Get owner Handle give owner_id
;
; Actually this function searches all the AvatarNode nodes with name owner_id
;
(define (get_owner_handle owner_id)
    (let ( (owner_handle (cog-node 'AvatarNode owner_id) )
         )

        (if (null? owner_handle)
            (print_debug_info INFO_TYPE_FAIL "get_owner_handle"
                              (string-append "Can not find the owner named " owner_id)
            )  

            (print_debug_info INFO_TYPE_SUCCESS "get_owner_handle"
                              (string-append "Found the owner named " owner_id)
            )
        );if

        owner_handle; return the owner Handle (note: it might be null)
    );let
);define 

;==============================================================================
;
; Add a Modulator given modulator_name and default_value
;
; The updater of the modulator is a combo script, named after modulator_name with suffix "Updater"
;
; Modulator is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "modulator_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "modulator_schema_name"
;         ListLink 
;             PET_HANDLE
;
(define (add_modulator modulator_name default_value)
    (let ( (handle_created (list) )
           (modulator_name (string-trim-both modulator_name) )         
         )

         (set! handle_created
               (SimilarityLink (cog-new-stv 1.0 1.0) (DEFAULT_AV)
                   (NumberNode (number->string default_value) (DEFAULT_STV) (DEFAULT_AV) )
                   (ExecutionOutputLink (DEFAULT_STV) (DEFAULT_AV) 
                        (GroundedSchemaNode (string-append modulator_name "Updater") 
                                            (DEFAULT_STV) (DEFAULT_AV)
                        )
                        (ListLink (DEFAULT_STV) (DEFAULT_AV) 
                            PET_HANDLE          
                        )
                   )
               );SimilarityLink
         );set!

         ; save the Handle to list 
         (set! modulator_list
               (assoc-set! modulator_list modulator_name handle_created)  
         );set!

         handle_created ; return the Handle
    );let
);define

;==============================================================================
;
; Add a DemandSchema/DemandValue given demand_name and default_value
;
; The updater of the demand value is a combo script, named after demand_name with suffix "Updater"

; DemandSchema/DemandValue is represented as:
;
; SimilarityLink (stv 1.0 1.0)
;     NumberNode: "demand_value"
;     ExecutionOutputLink
;         GroundedSchemaNode: "demand_schema_name"
;         ListLink
;             PET_HANDLE
;
; DemandValue is the output of DemandSchema.
;

(define (add_demand_schema demand_name default_value)
    (let ( (handle_created (list) )
           (demand_name (string-trim-both demand_name) )   
         )
         
         (set! handle_created
               (SimilarityLink (cog-new-stv 1.0 1.0) (DEFAULT_AV)
                   (NumberNode (number->string default_value) (DEFAULT_STV) (DEFAULT_AV) )
                   (ExecutionOutputLink (DEFAULT_STV) (DEFAULT_AV)
                        (GroundedSchemaNode (string-append demand_name "Updater") 
                                            (DEFAULT_STV) (DEFAULT_AV) 
                        )
                        (ListLink (DEFAULT_STV) (DEFAULT_AV)
                            PET_HANDLE 
                        )
                   )
               );SimilarityLink
         );set!

         ; save the Handle to list 
         (set! demand_schema_list         
               (assoc-set! demand_schema_list demand_name handle_created) 
         );set!

         handle_created ; return the Handle
   );let
);define

;==============================================================================
; 
; Add DemandGoal given demand_name, min_value, max_value and DemandSchema
; The PredicateNode added is usually named after demand_name with "Goal" suffix
;
; DemandGoal is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode: "demand_name_goal" 
;                        (SimpleTruthValue indicates how well the demand is satisfied)
;                        (ShortTermInportance indicates the urgency of the demand)
;     EvaluationLink
;         GroundedPredicateNode: "FuzzyWithin"
;         ListLink
;             NumberNode: "min_acceptable_value"
;             NumberNode: "max_acceptable_value"
;             SimilarityLink (stv 1.0 1.0)
;                 NumberNode: "demand_value"
;                 ExecutionOutputLink
;                     GroundedSchemaNode: "demand_schema_name"
;                     ListLink
;                         PET_HANDLE

(define (add_demand_goal demand_name min_acceptable_value max_acceptable_value)
    (let* (
            (handle_created (list) )    
            (demand_name (string-trim-both demand_name) )
            (demand_schema_handle (get_handle_from_list demand_schema_list demand_name) )           
          )

          (if (null? demand_schema_handle)
              (begin
                  (print_debug_info INFO_TYPE_FAIL "add_demand_goal"
                                    (string-append "Please add DemandSchema firstly "
                                                   "before adding DemandGoal "
                                                   "for Demand named " demand_name
                                    )
                  );print_debug_info

                  (exit -1)
              );begin

              (begin
                  (set! handle_created
                      (SimultaneousEquivalenceLink (DEFAULT_STV) (DEFAULT_AV)
                          (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)
                              (PredicateNode (string-append demand_name "Goal")
                                             (DEFAULT_STV) (DEFAULT_AV) 
                              )
                          );EvaluationLink

                          (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)
                              (GroundedPredicateNode "FuzzyWithin")  
                              (ListLink (DEFAULT_STV) (DEFAULT_AV)
                                  (NumberNode (number->string min_acceptable_value) 
                                              (DEFAULT_STV) (DEFAULT_AV) 
                                  )
                                  (NumberNode (number->string max_acceptable_value) 
                                              (DEFAULT_STV) (DEFAULT_AV) 
                                  )
                                  demand_schema_handle
                              );ListLink
                          );EvaluationLink
                      );EquivalenceLink
                  );set!

                  ; save the Handle
                  (set! demand_goal_list
                        (assoc-set! demand_goal_list demand_name handle_created) 
                  );set!
              );begin
          );if

          handle_created ; return the Handle
    );let*
);define

;==============================================================================
;
; Add Precondition given precondition_name,
; while precondition_name is usually rule_name wuth "Precondition" suffix
;
; Precondition is represented as:
;
; AtTimeLink
;     TimeNode CURRENT_TIMESTAMP
;     EvaluationLink
;         GroundedPredicateNode "precondition_name"
;         ListLink (empty)
; 

(define (add_precondition precondition_name)
    (let ( (precondition_name (string-trim-both precondition_name) )
         )
         (AtTimeLink (DEFAULT_STV) (DEFAULT_AV)
             (TimeNode CURRENT_TIMESTAMP (DEFAULT_STV) (DEFAULT_AV) ) 
             (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)
                 (GroundedPredicateNode precondition_name (DEFAULT_STV) (DEFAULT_AV) )
                 (ListLink (DEFAULT_STV) (DEFAULT_AV) )
             )
         );AtTimeLink
    );let
);define

;==============================================================================
;
; Split the action into two parts, function_name and parameters
;
; The format of of input action is something like  
;     "beg_to_owner('_*_' , 'self', beg_to_me('owner', 30), 40.7)"
;
; The basic idea is that
;     function_name is the substring of action before the frist '(' from left to right
;     parameters is the substring of action between the first '(' the last ')'
;
; So as an example, after calling function split_function_parameters with the action above, 
; you wil get a list contains splitted function_name and parentheses, that is
;
; (list "beg_to_owner" 
;       "'_*_' , 'self', beg_to_me('owner', 30), 40.7"
; )
;

(define (split_function_parameters action)
    (let ( (function_name "")
           (parameters "")
         )

         (let ( (left_bracket_index  (string-index action #\( ) ) ; Search the first '(' from left to right
                (right_bracket_index (string-rindex action #\) ) ); Search the first ')' from right to left
              )
        
              (cond
                   ;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                   ;
                   ; If both '(' and ')' are found
                   ;
                   ( (and left_bracket_index right_bracket_index)

                     ; Check there in nothing after the first ')' from right to left
                     (let ( (str_after_right_bracket (string-trim-both
                                                         (substring action 
                                                                    (+ right_bracket_index 1)
                                                         );substring
                                                     ) 
                            );str_after_right_bracket                         
                          )
                           
                          ; If there is something after the first ')' from right to left
                          (if (> 
                                  (string-length str_after_right_bracket)
                                  0
                              )

                              (begin
                                  (print_debug_info INFO_TYPE_FAIL "split_function_parameters" 
                                                    (string-append "We found an unexpected string '"
                                                                    str_after_right_bracket "' "
                                                                   "when parsing " action
                                                    )
                                  );print_debug_info

                                  (exit -1)
                              );begin
                          );if 
                     );let

                     ; Get the function name,
                     ; that is the substring before the first '(' from left to right
                     (set! function_name 
                           (string-trim-both
                               (substring action 0 left_bracket_index)
                           )
                     );set!  

                     ; Get parameters, 
                     ; that is the substring between '(' and ')'
                     (set! parameters
                           (string-trim-both
                               (substring action 
                                          (+ left_bracket_index 1)
                                          right_bracket_index
                               );substring  
                           )
                     );set!
                   )

                   ;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                   ;
                   ; If both '(' and ')' are missing,
                   ;
                   ; we guess the input action contains only function name, 
                   ; and the parameter is empty.
                   ; We don't recommend you to do this, while it works!
                   ;
                   ( (and 
                         (not left_bracket_index)
                         (not right_bracket_index)
                     );and
                        
                     (set! function_name (string-trim-both action) )
                     (set! parameters "")

                     (print_debug_info INFO_TYPE_WARN "split_function_parameters" 
                                       (string-append "Missing both '(' and ')' when parsing "
                                                       action
                                                      ". We strongly recommend you add '()' "
                                                      "even if there is none parameter."
                                       )
                     );print_debug_info
                   )

                   ;+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                   ;
                   ; If '(' or ')' is missing
                   ;
                   ( else

                     (if (not left_bracket_index)
                         ; If missing '('
                         (begin
                             (print_debug_info INFO_TYPE_FAIL "split_function_parameters"
                                               (string-append "Missing '(' when parsing " action)
                             )

                             (exit -1)
                         );begin

                         ; If missing ')'
                         (begin
                             (print_debug_info INFO_TYPE_FAIL "split_function_parameters"
                                               (string-append "Missing ')' when parsing " action)
                             )
                             (exit -1)
                         );begin                     
                     );if
                   )
              );cond
         );let

         ; Check the function name is not empty
         (if (=
                 (string-length function_name)
                 0
             )

             (begin
                 (print_debug_info INFO_TYPE_FAIL "split_function_parameters"
                                   (string-append "The function name is missing when parsing " action)
                 )

                 (exit -1)
             );begin
         );if
     
         (list function_name parameters) ; return the function name and parameters
    );let  
);define

;==============================================================================
;
; Split parameters(a string) into a list of parameters,
; that is each element of the list is a parameter
;
; For instance, after executing
;     (split_parameters "'_*_' , 'self', beg_to_me('owner', 30), 40.7")
;
; You'll get 
;     (list "'_*_'"
;           "'self'"
;           "beg_to_owner('owner, 30')"
;           "40.7"
;     )
;
; The parameters of the action is the substring between the first '(' and the last ')'
; For example, the parameters of the action below
;     "beg_to_owner('_*_' , 'self', beg_to_me('owner', 30), 40.7)"
; is "'_*_' , 'self', beg_to_me('owner', 30), 40.7"
;
; Using function split_function_parameters, we can easily get the parameters given action
;

(define (split_parameters parameters)
    (let* ( (parameter_list (list) ) ; return value 
            (parameters (string-trim-both parameters) ) ; trim the parameters
            (splited_string_list (string-split parameters #\,) ) ; split the parameters by delimiter ','
            (splited_string "")      ; a string in splited_string_list, used by loop_splited_string 
            (left_bracket_count 0)   ; indicates the number of left/right brackets, 
            (right_bracket_count 0)  ; used by parentheses matching Check
            (parameter "")           ; promising parameter, used by loop_splited_string
          )

          ; Check parameters is not empty
          (if (>
                  (string-length parameters)
                  0
              )

              ; Fristly, split parameters into a list of strings by delimiter ','
              ; And then process the splited strings one by one.
              ;
              ; The basic idea is that:
              ;     if the splited string contains matched parentheses, 
              ;     then it is a complete parameter string, should be added to the parameter_list,
              ;     else, we append the next splited string to it trying to resolve unmatched parentheses.
              ;     At the end of the processing, we would check if the parentheses are matched.
              (let loop_splited_string ( )

                    (if (not
                            (null? splited_string_list) 
                        )

                        ; If the splited_string_list is not empty, 
                        ; then process the splited_string one by one
                        (begin
                            ; Get the first splited_string that hasn't been processed
                            ; and remove it from the splited_string_list
                            (set! splited_string (car splited_string_list) )
                            (set! splited_string_list (cdr splited_string_list) )

                            ; Update the promising parameter
                            (set! parameter
                                  (string-append parameter splited_string)
                            );set!

                            ; Update left/right bracket counts
                            (set! left_bracket_count
                                  (+
                                      left_bracket_count
                                      (string-count splited_string #\( )
                                  )
                            );set!

                            (set! right_bracket_count
                                  (+
                                      right_bracket_count
                                      (string-count splited_string #\) )
                                  )
                            );set!

                           (if (= left_bracket_count right_bracket_count)
                                   
                                ; If the parentheses are matched, then a complete parameter is found.
                                ;
                                ; Add the complete parameter to parameter_list, 
                                ; and reset the temporal promising parameter to empty string
                                (begin
                                    (set! parameter_list
                                          (append parameter_list (list parameter) )
                                    );set!    
                                    
                                    (set! parameter "")
                                );begin

                                ; If the parentheses are not matched, 
                                ; whch means we are traped in incomplete subfunction calling 
                                ;
                                ; Append a comma at the end of current promising parameter,
                                ; and expect more characters afterwards for solving unmatched parentheses
                                (set! parameter
                                      (string-append parameter ",")
                                );set!
                            );if

                            ; Process the next splited_string
                            (loop_splited_string)
                        );begin                      

                        ; At the end of the processing (loop_splited_string), 
                        ; we would check if the parentheses are matched.
                        ;
                        ; If the parentheses are not matched
                        (if (>
                                left_bracket_count
                                right_bracket_count
                            )

                            ; If missing right bracket
                            (begin
                                (print_debug_info INFO_TYPE_FAIL "split_parameters" 
                                                  (string-append "Missing "
                                                                 (number->string
                                                                     (- left_bracket_count 
                                                                        right_bracket_count
                                                                     )
                                                                 ) 

                                                                 " ')' " "when processing " parameters
                                                  );string-append                  
                                );print_debug_info

                                (exit -1)
                            );begin

                            ; If missing left bracket
                            (if (>
                                left_bracket_count
                                right_bracket_count
                                )                            
                                (begin
                                    (print_debug_info INFO_TYPE_FAIL "split_parameters" 
                                                      (string-append "Missing "
                                                                     (number->string
                                                                         (- right_bracket_count 
                                                                            left_bracket_count
                                                                         )
                                                                     ) 
                                                                     " ')' " "when processing " parameters
                                                      );string-append                  
                                    );print_debug_info

                                    (exit -1)
                                );begin
                            );if                            
                        );if
                    );if  
              );let loop_splited_string
          );if

          parameter_list ; return the list of parameters
    );let*
);define

;==============================================================================
;
; Add action to AtomSpace given action, something like 
;     "beg_to_owner('_*_' , 'self', beg_to_me('owner', 30), 40.7)"
;
; Action is represented as:
;
; ExecutionLink
;     GroundedSchemaNode "schema_name"
;     ListLink
;         Node:params
;

(define (add_action action)
    (let ( (handle_created (list) )  ; return value
           (function_name "")        ; function name after calling split_function_parameters
           (parameters "")           ; parameters after calling split_function_parameters
           (parameter_list (list) )  ; splitted parameters after calling split_parameters
           (parameter_handle_list (list) )  ; used to create a ListLink in AtomSpace
           (WILD_CARD_STR "'_*_'")
         )

         ; Split the action into two parts, function_name and parameters
         (let ( (splited_function_parameters_list (split_function_parameters action) )
              )
              
              (begin
                  (set! function_name (list-ref splited_function_parameters_list 0) )
                  (set! parameters (list-ref splited_function_parameters_list 1) )
              )
         );let
        
         ; Split the parameters (a string) into a list of strings 
         (set! parameter_list (split_parameters parameters) )
 
         ; Process parameter in parameter_list one by one,
         ; create nodes for each parameter, 
         ; and add them to parameter_handle_list, which is used for creating a ListLink in AtomSpace
         (let loop_parameter ( (parameter "")
                             ) 

              (if (not (null? parameter_list)
                  )

                  (begin
                      ; Get the first parameter from parameter_list and remove it from the list
                      (set! parameter (car parameter_list) )
                      (set! parameter_list (cdr parameter_list) )

                      ; Trim the parameter
                      (set! parameter (string-trim-both parameter) )
                      
                      ; Create a Node for the parameter 
                      ; and add the Handle to parameter_handle_list
                      (cond
                          ( (string=? parameter WILD_CARD_STR)

                            (set! parameter_handle_list
                                  (append parameter_handle_list
                                          (list 
                                              (VariableNode "VariableNode" (DEFAULT_STV) (DEFAULT_AV) )
                                          )
                                  );append
                            );set!
                          )

                          ( (string=? parameter "'self'")

                            (set! parameter_handle_list                            
                                  (append parameter_handle_list 
                                      (list PET_HANDLE)
                                  )
                            );set!
                          )

                          ( (string=? parameter "'owner'")

                            (set! parameter_handle_list 
                                  (append parameter_handle_list 
                                          (list OWNER_HANDLE)
                                  )
                            );set!
                          )
                          
                          ( (string->number parameter)
                          
                            (set! parameter_handle_list                             
                                  (append parameter_handle_list
                                          (list 
                                              (NumberNode parameter (DEFAULT_STV) (DEFAULT_AV) )
                                          )
                                  );append
                            );set!
                          )

                          ( (string-contains parameter "(")
                            
                            (set! parameter_handle_list 
                                  (append parameter_handle_list
                                          (list 
                                              (add_action parameter)
                                          )
                                  );append
                            );set!
                          ) 

                          ( else

                            (let* ( (parameter_length (string-length parameter) )
                                    (parameter (substring parameter
                                                          1
                                                          (- parameter_length 1)
                                               ) 
                                    ); Strip ' at both ends of parameter
                                  )

                                  (set! parameter_handle_list 
                                        (append parameter_handle_list
                                                (list 
                                                    (WordNode  parameter (DEFAULT_STV) (DEFAULT_AV) )
                                                )
                                        )
                                  );set!
                            );let*
                          )
                      );cond

                      ; Process the next parameter
                      (loop_parameter parameter)
                  );begin
              );if
         );let loop_parameter        

         (set! handle_created
               (ExecutionLink (DEFAULT_STV) (DEFAULT_AV)
                   (GroundedSchemaNode function_name (DEFAULT_STV) (DEFAULT_AV) )
                   (apply cog-new-link 
                          (append (list 'ListLink)
                                  parameter_handle_list
                                  (list (DEFAULT_STV) )
                                  (list (DEFAULT_AV) )
                          )
                   );apply
               );ExecutionLink
         );set!

         handle_created ; return the Handle created
    );let  
);define

;==============================================================================
;
; Add strengths
;
; It receives a string of strengths, and then creates and returns corresponding CompositeTruthValue 
; The strengths string input is something like;
;     PLAYING_MODE=0.7, SCAVENGER_HUNT_MODE=0, LEARNING_MODE=0
;
; TODO: This function would be replaced by something like add_rule_true_value

(define (add_strengths strengths)
    (let ( (composite_truth_value_created (list) )
         )

         ; 
         (let ( (splitted_list_by_comma (string-split strengths #\,) )
                (splitted_string_by_comma "")
                (strength_pair #\null)
                (mode_name "")
                (strength 0)
              )
              
              (let loop_splitted_string_by_comma ()
                   (if (null? splitted_list_by_comma)
                       (begin
                           (set! splitted_string_by_comma (car splitted_list_by_comma) )
                           (set! splitted_list_by_comma (cdr splitted_list_by_comma) )

                           (set! strength_pair (string-split strengths #\=) )

                           (set! mode_name 
                                 (string-trim-both 
                                     (list-ref string_pair 0) 
                                 )
                           );set!

                           (set! strength 
                                 (string->number?
                                     (list-ref string_pair 1) 
                                 )
                           );set!

                           (if (= (length string_pair)
                                  2
                               )
                            


                               (cog-new-vh "CONTEXTUAL" 
                                   (ConceptNode mode_name (DEFAULT_AV) ) 
                               )
  
                               (cog-new-stv strength 1.0)
 
                           );if

                           (loop_splitted_string_by_comma)  

                       );begin
                   );if
              );let loop_splitted_string_by_comma
          
         )

         composite_truth_value_created     

    );let*
);define

;==============================================================================
;
; Add rule to AtomSpace given rule_name, strengths, action and demand_name
; 
; Rule is represented as:
;
; PredictiveImplicationLink (CompositeTruthValue indicates the mode strengths)
;     AndLink
;         AtTimeLink
;             TimeNode CURRENT_TIMESTAMP
;             EvaluationLink
;                 GroundedPredicateNode "precondition_name"
;                 ListLink (empty)
;             ExecutionLink
;                 GroundedSchemaNode "schema_name"
;                 ListLink
;                     Node:params ...
;                     ...
;     AtTimeLink            
;         TimeNode CURRENT_TIMESTAMP
;         EvaluationLink
;             PredicateNode: "demand_name_goal" 
;                            (SimpleTruthValue indicates how well the demand is satisfied)
;                            (ShortTermInportance indicates the urgency of the demand)
;

; Add rule (a PredictiveImplicationLink) to AtomSpace given Handles of Precondition, Action and DemandGoal
(define (add_rule tv_handle precondition_handle action_handle demand_goal_predicate_handle)
    (PredictiveImplicationLink (DEFAULT_STV) tv_handle ; TODO: use CompositeTruthValue later
        (AndLink (DEFAULT_STV) (DEFAULT_AV)
            precondition_handle  
            action_handle
        )  

        (AtTimeLink
            (TimeNode CURRENT_TIMESTAMP)
            (EvaluationLink
               demand_goal_predicate_handle
            )
        );AtTimeLink
    );PredictiveImplicationLink
);define

; Add Rule to AtomSpace, users should use this function directly to add Rules to AtomSpace
(define (rule tv_handle precondition_name action demand_name)
    (let* ( (handle_created (list) ) 

            (precondition_name (string-trim-both precondition_name) )
            (action (string-trim-both action) )
            (demand_name (string-trim-both demand_name) )

            (precondition_handle (add_precondition precondition_name) )
            (action_handle (add_action action) )

            (demand_goal_predicate_handle 
                (cog-node 'PredicateNode (string-append demand_name "Goal") )
            )
          )

          (if (null? demand_goal_predicate_handle)
              ; If the corresponding DemandGoal is not found
              (begin
                  (print_debug_info INFO_TYPE_FAIL "rule"
                                    (string-append "Can not find required DemandGoal. " 
                                                   "You should add DemandGoal for " demand_name "firstly, "
                                                   " before adding Rule " rule_name
                                    )
                  );print_debug_info

                  (exit -1)
              );begin

              ; If the required DemandGoal is found
              (begin
                  (set! handle_created
                        (add_rule tv_handle 
                                  precondition_handle 
                                  action_handle 
                                  demand_goal_predicate_handle
                        );add_rule
                  ) 
              );begin
          );if

          handle_created
    );let*
);define

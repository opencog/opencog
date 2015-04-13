;
; @file embodiment/unity_stimulus_rules.scm
;
; @author Troy Huang <huangdeheng@gmail.com>
; @date   2011-09-07
;
; @brief Stimulus is an external signal receiving from the environment that may 
; lead to the change of agent's internal state(i.e. emotional state).
; Currently, stimulus can be an event or an action performed by other agents.
; e.g. A punched B, from the perspective of B, a nasty action has been performed 
; which may make it angry. But from the perspective of other agent, say C, this 
; is an event that it perceived. If B is a good friend of C, C may also gets 
; angry.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Predefined attitude nodes
;
(define ANGRY_HANDLE (ConceptNode "angry"))
(define THANKFUL_HANDLE (ConceptNode "thankful"))
(define LOVE_HANDLE (ConceptNode "love"))

(define TIME_NODE_TYPED_VARIABLE_LINK
    (TypedVariableLink
        (VariableNode "$var_time_node") 
        (VariableTypeNode "TimeNode")
    )
)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
; Add a stimulus precondition as a trigger of some rule
; @param stimulus_pred_handle A predicate node handle representing certain 
;                           attribute of a stimulus.
; @param stimulus_instance Handle of an stimulus instance
; @param arguments List of arguments that may contain the value of given 
;               attribute and other stuffs, depending on the format of stimulus 
;               attribute.
;
; @note Usually, the stimulus instance handle would be replaced by a variable node
;       if the function is invoked in the definition of a Rule.
;       For example, following is one of its Usage:
;           (add_stimulus_precondition
;               (PredicateNode "touch:target")
;               (VariableNode "$var_stimulus")
;               (AvatarNode "Player")
;           )
;       This precondition will be satisfied when a "touch" stimulus fires with 
;       its target the "Player" avatar.
;       
(define (add_stimulus_precondition stimulus_pred_handle stimulus_instance . arguments)
    (EvaluationLink
        stimulus_pred_handle
        (ListLink
            stimulus_instance
            ;(map-in-order
            ;    (lambda (argument)
            ;        (if (cog-atom? argument)
            ;            argument
            ;        )
            ;    )
            ;    arguments
            ;);map-in-order
            (apply parse_arguments arguments)
        )
    )
)

; Add an attitude towards certain role that maybe other agent or object.
; @note To generate an attitude is a kind of response.
(define (add_attitude target_handle attitude_handle strength) 
    (AtTimeLink (stv strength 1.0)
        ; In order to take timestamp into consideration, an at time Link is 
        ; added here. The variable time node would be replaced by a real 
        ; timestamp when applying the rule onto certain stimulus instance.
        (VariableNode "$var_time_node")

        (EvaluationLink (stv strength 1.0)
            (PredicateNode "attitude_towards")
            (ListLink 
                ; self node
                PET_HANDLE
                ; target node, usually a variable in a rule.
                target_handle
                ; attitude node
                attitude_handle
            ) 
        )
    );AtTimeLink
)

; Add a rule for agent to handle the stimulus.
; @param preconditions The preconditions to trigger the response, they are usually
; wrapped in a logic link(AndLink, OrLink etc.) to represent their relationships.
; @param response The response to be triggerred.
;
; @note Attention! The variables used in certain rule should be declared in the 
;       scope of that rule. e.g.
;
;       (define SomeRule
;           (ImplicationLink 
;               Preconditions with variable $var
;               Response with variable $var 
;           )
;       )
;
;       Declare $var in a scope.
;       (ForAllLink (stv 1 1)
;           (VariableNode "$var")
;           SomeRule
;       )
(define (add_stimulus_rule preconditions response)
    (ImplicationLink (cog-new-av 1 1 1)
        preconditions
        response
    )
)

; Get precondition/ response from a given rule
(define (get_preconditions_from_rule rule)
    (list-ref (cog-outgoing-set rule) 0)
)

(define (get_response_from_rule rule)
    (list-ref (cog-outgoing-set rule) 1)
)

; Get the prototype of a stimulus instance. 
; The relationship between instance and its prototype is usually defined by an 
; 'InheritanceLink'. e.g.
;       (InheritanceLink 
;           (ConceptNode "touch22")
;           (ConceptNode "touch")
;       )
; "touch22" is a "touch" instance in this case.
(define (get_stimulus_prototype stimulus_handle)
    (cog-bind
        (BindLink
            (VariableNode "$var_prototype")

            (ImplicationLink 
                ; Pattern
                (InheritanceLink
                    stimulus_handle
                    (VariableNode "$var_prototype")
                )

                ; Result
                (VariableNode "$var_prototype")
            )
        )
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Maintain a map from action to rules
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

; Declare a dummy map, rules are added in the end of file.
(define stimulus_rules_map
    (list)
)

; Get a list of rules that maps to given action name
(define (get_stimulus_rules_from_map stimulus_name)
    (if 
        (list? (assoc stimulus_name stimulus_rules_map))
        (car (cdr (assoc stimulus_name stimulus_rules_map)))
        (list)
    )
)

; Find stimulus rules according to stimulus instance handle.
; 
; @param stimulus_handle The handle of stimulus instance
(define (find_stimulus_rule stimulus_handle)
    (let 
        ( 
            ; A stimulus instance might have multiple prototypes
            ( stimulus_prototype_handle_list
              (unpack_query_result (get_stimulus_prototype stimulus_handle))
            )
            ; rule list to return
            ( rule_list (list) )
        )
        
        ; Push all rules that mapping to the stimulus into a list
        (map-in-order
            (lambda (stimulus_prototype_handle)
                (set! rule_list
                    (append 
                        rule_list 
                        (get_stimulus_rules_from_map (cog-name stimulus_prototype_handle))
                    )
                )
            );lambda
            stimulus_prototype_handle_list
        );map-in-order
        ;(display rule_list) (newline)
        rule_list
    );let
)

; The interface method to be invoked by c++ code. 
; This method can find rules according to the stimulus instance handle and try to apply them 
; on the instance. If all the preconditions are satisfied, the instance can successfully 
; fire the rules. 
; @param stimulus_handle The handle of stimulus instance, i.e. (ConceptNode "touch22").
(define (apply_stimulus_rule stimulus_handle)
    (let
        ; The result set to be returned
        ( 
            (stimulus_atom (cog-atom stimulus_handle))
            (result_list (list)) 
        )

        (map-in-order
            (lambda (rule)
                ; Get the variables to apply on the rule, learned from:
                ; opencog/embodiment/action_selection.scm by Zhenhua CAI
                (let ( (variables (list)) )
                    (begin 
                        (let search_variables 
                            ( (incomings (cog-incoming-set rule) )
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
                            ); if
                        );let 

                        ; Apply the rule if the variables are not null
                        (if (not (null? variables))
                            (begin
                                ; Reassemble the variables by adding a time node variable.
                                (if (equal? (cog-type variables) 'ListLink)

                                    (set! variables
                                        (ListLink 
                                            (apply parse_arguments (cog-outgoing-set variables))
                                            TIME_NODE_TYPED_VARIABLE_LINK
                                        )
                                    )

                                    (set! variables
                                        (ListLink
                                            variables
                                            TIME_NODE_TYPED_VARIABLE_LINK
                                        )
                                    )
                                 );if

                                 ;(display "Apply rule") (newline)

                                 (let* 
                                    (
                                        (preconditions (list-ref (cog-outgoing-set rule) 0))
                                        (response (list-ref (cog-outgoing-set rule) 1))
                                        ; Logical relationship among preconditions.
                                        (preconditions_logic_type (cog-type preconditions))
                                        (new_preconditions 
                                            (cog-new-link preconditions_logic_type
                                                (apply parse_arguments (cog-outgoing-set preconditions))
                                                (AtTimeLink (stv 1 1)
                                                    (VariableNode "$var_time_node")
                                                    (VariableNode "$var_stimulus")
                                                )
                                            )
                                        )
                                    )

                                    ;(display preconditions_logic_type) (newline)
                                    ;(display variables) (newline)
                                    (display new_preconditions) (newline)

                                    (set! result_list 
                                         (append 
                                             result_list
                                             (query_atom_space
                                                 (BindLink
                                                     variables
                                                     (ImplicationLink
                                                        new_preconditions
                                                        response
                                                     )
                                                 )
                                             )
                                         )
                                     )
                                     (cog-delete new_preconditions)
                                 );let*
                            );begin
                        );if
                    );begin
                );let
            );lambda

            ; Get the rule set by given stimulus instance handle
            (find_stimulus_rule stimulus_atom)
        );map-in-order

        result_list
    );let
)

;|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Helper functions
;
;|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

; Query the avatar's attitudes towards some agent 
; Attention, this method may return all the historical attitudes towards given 
; agent. If you just want the latest attitude, use the 
; "get_latest_attitude_towards" method instead.
(define (get_attitude_towards agent_handle)
    (query_atom_space
        (BindLink
            (VariableNode "$var_attitude")

            (ImplicationLink 
                ; Pattern
                (EvaluationLink
                    (PredicateNode "attitude_towards")
                    (ListLink
                        PET_HANDLE
                        agent_handle
                        (VariableNode "$var_attitude")
                    )
                )

                ; Result
                (EvaluationLink
                    (PredicateNode "attitude_towards")
                    (ListLink
                        PET_HANDLE
                        agent_handle
                        (VariableNode "$var_attitude")
                    )
                )
            );ImplicationLink
        );BindLink
    )
)

(define (get_attitude_towards_with_at_time_link agent_handle)
    (query_atom_space
        (BindLink
            (ListLink
                (TypedVariableLink
                    (VariableNode "$var_time_node")
                    (VariableTypeNode "TimeNode")
                )
                (VariableNode "$var_attitude")
            )

            (ImplicationLink 
                ; Pattern
                (AtTimeLink
                    (VariableNode "$var_time_node")
                    (EvaluationLink
                        (PredicateNode "attitude_towards")
                        (ListLink
                            PET_HANDLE
                            agent_handle
                            (VariableNode "$var_attitude")
                        )
                    )
                )

                ; Result
                (AtTimeLink
                    (VariableNode "$var_time_node")
                    (EvaluationLink
                        (PredicateNode "attitude_towards")
                        (ListLink
                            PET_HANDLE
                            agent_handle
                            (VariableNode "$var_attitude")
                        )
                    )
                )
            );ImplicationLink
        );BindLink
    )
)

; Query the avatar's latest attitude towards some agent.
(define (get_latest_attitude_towards agent_handle)
    (let
        (
            (attitude_at_time_link_list 
                    (get_attitude_towards_with_at_time_link agent_handle)) 
            (latest_attitude_pred_link (list))
            (latest_timestamp (list))
        )

        (map-in-order
            (lambda (attitude_at_time_link)
                (let* 
                    (
                        (time_node (list-ref (cog-outgoing-set attitude_at_time_link) 0) )
                        (timestamp (string->number (cog-name time_node) ) )
                    )
      
                     (if (or (null? latest_timestamp)
                             (> timestamp latest_timestamp)
                         )
                         
                         (begin
                             (set! latest_attitude_pred_link 
                                (list-ref (cog-outgoing-set attitude_at_time_link) 1)) 
                             (set! latest_timestamp timestamp)
                         )
  
                     ); if 
                )
            )
            attitude_at_time_link_list
        );map-in-order

        latest_attitude_pred_link
    )
)

; Query the attribute of a given action instance.
; The action attribute is stored in following format:
;       EvaluationLink
;           PredicateNode "action:attribute"
;           ListLink
;               ConceptNode "action instance"
;               SomeNode "attribute value"
(define (get_action_attribute_value action_handle attribute_name)
    (let* 
        (
            ( action_prototype_list
              (unpack_query_result (get_stimulus_prototype action_handle))
            )
            ( action_attribute
              (if (null? action_prototype_list)
                  (string-append "undefined:" attribute_name)
                  (string-append (cog-name (car action_prototype_list)) ":" attribute_name) 
              )
            )
        )
        
        ; Begin the query procedure

        ; Return a scheme list instead of a ListLink
        (unpack_query_result
            ; Invoke pattern matching.
            (cog-bind
                (BindLink
                    ; Variables
                    (VariableNode "$var_any")

                    (ImplicationLink
                        ; Query pattern
                        (EvaluationLink
                            (PredicateNode action_attribute)
                            (ListLink
                                action_handle
                                (VariableNode "$var_any")
                            )
                        )
                        
                        ; Query result
                        (VariableNode "$var_any")
                    )
                );BindLink
            );cog-bind
        )
    );let
)

;|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Specific rules region
;
;|||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

;
; Touch action has following parameters:
;   @target
;   @actor
;   @force
;   @result
;
(define TouchHeavilyRule
    (add_stimulus_rule
        ; Add preconditions with logical relationship
        (AndLink
            ;(InheritanceLink
            ;    (VariableNode "$var_stimulus")
            ;    (ConceptNode "touch")
            ;)

            (add_stimulus_precondition
                (PredicateNode "touch:actor")
                (VariableNode "$var_stimulus")
                (VariableNode "$var_actor")
            ) 

            (add_stimulus_precondition
                (PredicateNode "touch:target")
                (VariableNode "$var_stimulus")
                "'self'"
            )

            (add_stimulus_precondition
                (PredicateNode "touch:force")
                (VariableNode "$var_stimulus")
                (ConceptNode "extremely_high")
            )
        );AndLink

        ; Add response
        (add_attitude 
            (VariableNode "$var_actor")
            ANGRY_HANDLE
            1.0
        )
    )
)

; Add scope for touch heavily rule
(ForAllLink (cog-new-av 1 1 1)
    (ListLink
        (TypedVariableLink
            (VariableNode "$var_stimulus") 
            (VariableTypeNode "ConceptNode")
        )

        (VariableNode "$var_actor") 
    )

    TouchHeavilyRule
);ForAllLink

(define stimulus_rules_map
    (list
        ; Each time when a new rule is added, it should be appended in the map
        ; The format is: (action_name (list rule1 rule2 ...))
        (list "touch" (list TouchHeavilyRule) )
        ; TODO add new rules if there are new stimulus to be applied.
        ; (list "kick" SomeKickRule)
        ; (list "hug" SomeHugRule)
    )
)

stimulus_rules_map

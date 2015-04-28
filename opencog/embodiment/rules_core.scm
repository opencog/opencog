;
; @file embodiment/rules_core.scm
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-11-28
;
; Scheme core functions for adding Modulators, Demands and Rules etc. into AtomSpace
;

;******************************************************************************
;
; How do OpenPsi components represented in AtomSpace?
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Modulator is represented as:
;
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;
;     SimilarityLink
;         NumberNode: "modulator_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "modulator_schema_name"
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; DemandSchema/DemandValue is represented as:
; 
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;
;     SimilarityLink 
;         NumberNode: "demand_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "demand_schema_name"
;
; DemandValue is the output of DemandSchema.
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Each none grounded goal or precondition should have a corresponding 
; GroundedPredicateNode to check if the goal or precondition has been achieved 
; or not. They are related via an SimultaneousEquivalenceLink as follows:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode "none_grounded_goal_or_precondition_name"
;         ListLink
;             ...
;
;     EvaluationLink
;         GroundedPredicateNode "updater_schema_name"
;         ListLink
;             ...
;
; A special case is the DemandGoal, which uses a "fuzzy_within" scheme function
; to calculate its truth value. While the XxxDemandUpdater is a scheme function 
; of updating the demand level (not the truth value of it). 
;
; Take CertaintyDemand as an example, the CertaintyDemandUpdater is responsible 
; for updating the certainty level of the agent via a bunch of information stored 
; in AtomSpace, while the truth value of the CertaintyDemandGoal is calculated 
; via fuzzy_within function, which would evaluate how well the certainty level is 
; within the suitable range [min_acceptable_value, max_acceptable_value]. 
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode "XxxDemandGoal" 
;     EvaluationLink
;         GroundedPredicateNode "fuzzy_within"
;         ListLink
;             NumberNode: min_acceptable_value
;             NumberNode: max_acceptable_value
;             ExecutionOutputLink
;                 GroundedSchemaNode "XxxDemandUpdater"
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Finally, a Psi Rule is represented as:
;
; ImplicationLink (truth value indicates the probability to be selected while planning)
;     AndLink
;         AndLink
;             precondition_1 (truth value indicates how well the precondition or subgoal is satisfied)
;             subgoal_1
;             ...
;
;         ExecutionLink (truth value indicates whether the action has been done successfully)
;             action 
;
;     EvaluationLink (truth value indicates how well the goal is satisfied)
;         goal 
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Reference:
;
;     http://wiki.opencog.org/w/OpenCogPrime:FunctionNotation
;     http://wiki.opencog.org/w/TimeServer
;
;******************************************************************************

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Information type used by print_debug_info function
;
(define INFO_TYPE_SUCCESS "SUCCESS")
(define INFO_TYPE_FAIL    "FAIL")
(define INFO_TYPE_WARN    "WARN")

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Print some information for degugging, if you dislike them, just comment the body of the function
;
(define (print_debug_info info_type function_name info)
    (display (string-append info_type " in " function_name ": " info)
    )
    (newline)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add a Modulator given modulator_name and default_value
;
; The updater of the modulator is a combo script, named after modulator_name with suffix "Updater"
;
; Modulator is represented as:
;
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;     SimilarityLink 
;         NumberNode: "modulator_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "modulator_schema_name"
;

(define (add_modulator modulator_name default_value)
    (let ( (schema_handle (ExecutionOutputLink (stv 1.0 1.0) (cog-new-av 1 1 1) 
                              (GroundedSchemaNode (string-append (string-trim-both modulator_name) "Updater") ) 
                          );ExecutionOutputLink
           );schema_handle
         )

         (AtTimeLink (stv 1.0 1.0)
             (TimeNode "0")

             (SimilarityLink
                 (NumberNode (number->string default_value) )
                 schema_handle
             );SimilarityLink
         );AtTimeLink

         schema_handle

    );let
);define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add a DemandSchema/DemandValue given demand_name and default_value, 
; return the handle to ExecutionOutputLink, which would be used by 'fuzzy_within' function
;
; The updater of the demand value is a combo script, named after demand_name with suffix "Updater"
;
; DemandSchema/DemandValue is represented as:
;
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;     SimilarityLink 
;         NumberNode: "demand_value"
;         ExecutionOutputLink 
;             GroundedSchemaNode: "demand_schema_name"
;
; DemandValue is the output of DemandSchema.
;

(define (add_demand_schema demand_name default_value)
    (let ( (schema_handle (ExecutionOutputLink (cog-new-av 1 1 1)
                              (GroundedSchemaNode (string-append (string-trim-both demand_name) "Updater") )
                          );ExecutionOutputLink
           );schema_handle    
         )    

        (AtTimeLink (stv 1.0 1.0)
            (TimeNode "0")

            (SimilarityLink
                (NumberNode (number->string default_value) )
                schema_handle
            );SimilarityLink
        );AtTimeLink

        schema_handle

    );let
);define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
; 
; Connect a none grounded goal and a grounded predicate node. 
; 
; Each none grounded goal or precondition should have a corresponding 
; GroundedPredicateNode to check if the goal or precondition has been achieved or
; not. They are related via an SimultaneousEquivalenceLink as follows:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         PredicateNode "none_grounded_goal_or_precondition_name"
;         ListLink
;             ...
;
;     EvaluationLink
;         GroundedPredicateNode "updater_schema_name"
;         ListLink
;             ...
;

(define (connect_goal_updater goal_evaluation_link goal_truth_value_updater_evaluation_link)
    (SimultaneousEquivalenceLink (cog-new-stv 1.0 1.0) (cog-new-av 1 1 1)
        goal_evaluation_link
        goal_truth_value_updater_evaluation_link
    ) 
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; This helper function will return a list that contains a bunch of Atoms for combo function arguments. 
;
; Example:
;
;     For a list of arguments
;         ( "'self'"  "$var_entity"  40.7 "'owner'" EvaluationLinkHandle )
;
;     After calling this function, you will get a list containing Handles to corresponding Atoms
;
;         ( PET_HANDLE 
;           VariableNode "$var_entity"
;           NumberNode "40.7"
;           OWNER_HANDLE
;           EvaluationLinkHandle
;         )
;

(define (parse_arguments . arguments)
    (map-in-order
        (lambda (argument)

              ; Create a corresponding Node for the argument
              (cond

                  ( (cog-atom? argument) 
                    argument
                  )

                  ( (number? argument)
                    (NumberNode (number->string argument) )
                  )   

                  ( (equal? (string-contains argument "'") #f)
                    (VariableNode (string-trim-both argument) )
                  )

                  ( (string=? (string-trim-both argument) "'self'")
                    PET_HANDLE
                  )

                  ( (string=? (string-trim-both argument) "'owner'")
                    OWNER_HANDLE
                  )
                  
                  ( else
                      (let* ( (argument (string-trim-both argument) )
                              (argument_length (string-length argument) )
                              (argument (substring argument
                                                   1
                                                   (- argument_length 1)
                                        ) 
                              ); Strip ' at both ends of argument
                            )

                           (WordNode  argument)
                      );let*
                  );else

              );cond
        
        );lambda

        arguments
    );map-in-order
);define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Goal to AtomSpace. 
;
; A Goal is an EvaluationLink with a PredicateNode or GroundedSchemaNode, 
; which can be represented as below:
;
; EvaluationLink
;     PredicateNode "ungrounded_goal_name"
;     ListLink
;         Node:arguments
;         ...
;
; or 
;
; EvaluationLink
;     GroundedPredicateNode "grounded_goal_name"
;     ListLink
;         Node:arguments
;         ...
;
; After you add an ungrounded goal, you should connect it with a grounded
; PredicateNode via 'connect_goal_updater' function

(define (add_goal pred_or_gpn_handle . arguments)
    (EvaluationLink (cog-new-av 1 1 1)
        pred_or_gpn_handle   

        (let ( (argument_list (apply parse_arguments arguments) )
             )
             (if (null? argument_list)
                 (list)
                 (ListLink argument_list)
             )
        )
    )
)

; goal updater is responsible for updating the truth value of ungrounded goal. 
(define (add_goal_updater gpn_handle . arguments)
    (add_goal gpn_handle arguments) 
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Precondition to AtomSpace, which are represented as:
;
; AndLink
;     EvaluationLink
;         PredicateNode "sub_goal_name_1"
;         ListLink
;             ...
;     EvaluationLink
;         GroundedPredicateNode "gpn_sub_goal_name_2"
;             ...
;     ...        
;
; Technically speaking, there's no distinction between goal and precondition, 
; because a goal in one psi rule may serves as the precondition in another
; rule, vice versa.  
;

(define (add_precondition pred_or_gpn_handle . arguments)
    (apply add_goal pred_or_gpn_handle arguments)     
)

; NULL_PRECONDITION is a dummy Precondition that is always satisfied!
; 
; It is used when the Precondition (in another word Contex) is not necessary in a Rule.
;
; Then the cognitive schematic 
;     Contex & Procedure ==> Goal
; is reduced to 
;     Procedure ===> Goal
;
; Do we really need this NULL_PRECONDITION? Who knowns!
;

(define NULL_PRECONDITION 
    (cog-set-tv! 
        (add_goal (PredicateNode "NoPrecondition") )
        (stv 1.0 1.0)
    )    
)

(define NULL_CONTEXT
    (cog-set-tv! 
        (add_goal (PredicateNode "NoContext") )
        (stv 1.0 1.0)
    )    
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Action to AtomSpace.
;
; An Action is an ExecutionLink with a schema node, such as GroundedSchemaNode 
; or SpeechActSchemaNode
;
; Its Truth Value can only be evaluated by corresponding combo script. 
; 
; It can be represented as follows:
;
; ExecutionLink
;     GroundedSchemaNode "schema_name"
;     ListLink
;         Node:arguments
;         ...
;
; or
;
; ExecutionLink
;     SpeechActSchemaNode "schema_name"
;     ListLink
;         Node:arguments
;         ...
;
; Note: ListLink is required. Even there's no argument, you should also put an 
;       empty ListLink within ExecutionLink. 
;       Othersise, calling
;           (query_atom_space_crisp (find_psi_action variables context action) )
;           in 'opencog/embodiment/action_selection.scm' 
;       will discard all the actions without ListLink. This weird problem is 
;       caused by the behaviour of pattern matcher. 
;

(define (add_action schema_handle . arguments)
    (ExecutionLink (cog-new-av 1 1 1)
        schema_handle
        (ListLink (apply parse_arguments arguments) )

;        (let ( (argument_list (apply parse_arguments arguments) )
;             )
;             (if (null? argument_list)
;                 (list)
;                 (ListLink argument_list)
;             )
;        )
    )
)

; NULL_ACTION is a dummy Action that actually does nothing and its truth value is always ture!
; 
; It is used when the Action (in another word Procedure) is not necessary in a Rule.
;
; Then the cognitive schematic 
;     Contex & Procedure ==> Goal
; is reduced to 
;     Context ===> Goal
;
; For instance, in a Relation Rule, Action may be missing. 
; When someone attacks you, the Relation between you and the unfriendly guy would probably be Enemy, 
; without any Action you take. 
;
; Moreover, to some extend, do nothing is actualy do something :)
;

(define NULL_ACTION 
    (cog-set-tv! 
        (add_action (GroundedSchemaNode "DoNothing") )
        (stv 1.0 1.0)
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Rule (an ImplicationLink) to AtomSpace given Handles of Goal, Action and Preconditions
;
; For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
; And all these Preconditions should be grouped in an AndLink.
; If you want to use OrLink, then just split the Rule into several Rules.
; For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
;
; 1. Psi Rule is represented as follows:
;
; ImplicationLink (higher truth value means higher probability of selection when planning)
;     AndLink
;         AndLink
;             Preconditions
;         Action
;     Goal
;
; 2. Goal is represented as:
; 
; EvaluationLink
;     PredicateNode "goal_name_1"
;     ListLink
;         ...
;
; or
;
; EvaluationLink
;     GroundedPredicateNode "gpn_goal_name_2"
;     ListLink
;         ...
;
; 3. Preconditions are represented as:
;
; AndLink
;     EvaluationLink
;         PredicateNode "sub_goal_name_1"
;         ListLink
;             ...
;
;     EvaluationLink
;         GroundedPredicateNode "gpn_sub_goal_name_2"
;         ListLink
;             ...
;
; 4. Action is represented as: 
;
; ExecutionLink (truth value means if the action has been done successfully)
;     GroundedSchemaNode "schema_name_1"
;     ListLink
;         ...
;
; Note: For each goal or precondition, we can use PredicateNode or GroundedPredicateNode 
;       within EvaluationLink. The truth value means how well the goal or 
;       precondition are satisfied. The attention value means the urgency. 
; 
;       Each none grounded goal or precondition should have a corresponding 
;       GroundedPredicateNode to check if the goal or precondition has been 
;       achieved or not. They are related via an SimultaneousEquivalenceLink
;       as follows:
;
;       SimultaneousEquivalenceLink
;           EvaluationLink
;               PredicateNode "none_grounded_goal_or_precondition_name"
;               ListLink
;                   ...
;           EvaluationLink
;               GroundedPredicateNode "updater_schema_name"
;               ListLink
;                   ...
;

(define (add_rule truth_value goal_evaluation_link action_execution_link precondition_and_link)
    (ImplicationLink (cog-new-av 1 1 1) truth_value
        (AndLink
            precondition_and_link
            action_execution_link
        ) 

        goal_evaluation_link
    ) 
)


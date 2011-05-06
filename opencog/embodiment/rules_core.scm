;
; @file embodiment/rules_core.scm
;
; @author Zhenhua Cai <czhedu@gmail.com>
; @date   2011-03-11
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
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "modulator_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "modulator_schema_name"
;             ListLink (empty)
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; DemandSchema/DemandValue is represented as:
; 
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "demand_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "demand_schema_name"
;             ListLink (empty)
;
; DemandValue is the output of DemandSchema.
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; The connection of Demand and Goal is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "demand_name_goal" 
;         ListLink (empty)
;
;     EvaluationLink
;         GroundedPredicateNode: FuzzyWithin
;         ListLink
;             NumberNode: min_acceptable_value
;             NumberNode: max_acceptable_value
;             ExecutionOutputLink
;                 GroundedSchemaNode: "demand_schema_name"
;                 ListLink (empty)
;
;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Finally, Rule is represented as:
;
; PredictiveImplicationLink
;     AndLink
;         AndLink
;             EvaluationLink
;                 GroundedPredicateNode "precondition_1_name"
;                 ListLink
;                     Node:arguments
;                     ...
;             EvaluationLink
;                 PredicateNode         "precondition_2_name"
;                 ListLink 
;                     Node:arguments
;                     ...
;             ...
;                        
;         ExecutionLink
;             GroundedSchemaNode "schema_name"
;             ListLink
;                 Node:arguments
;                 ...
;
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "goal_name" 
;         ListLink
;             Node:arguments
;             ...
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
; Default Attention Values
;
; Settings here are the same with "AttentionValue.h" line 48-50
;
(define DEFAULT_STI 0)
(define DEFAULT_LTI 0)
(define DEFAULT_VLTI #f)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; These Default values are only used for loading Demand, Rules etc, and 
; has nothing to do with other C++ code
;
; Default Attention Value 
(define (DEFAULT_AV) 
    (cog-new-av DEFAULT_STI 1 DEFAULT_VLTI)
)

; Default Simple Truth Value
(define (DEFAULT_STV) 
    (cog-new-stv 0.01 0.01) 
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

(define CURRENT_TIMESTAMP "0")

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
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "modulator_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "modulator_schema_name"
;             ListLink (empty)
;

(define (add_modulator modulator_name default_value)
    (let ( (schema_handle (ExecutionOutputLink (stv 1.0 1.0) (DEFAULT_AV) 
                              (GroundedSchemaNode (string-append (string-trim-both modulator_name) "Updater") ) 
                              (ListLink)
                          );ExecutionOutputLink
           );schema_handle
         )

         (AtTimeLink (stv 1.0 1.0)
             (TimeNode CURRENT_TIMESTAMP)

             (SimilarityLink (cog-new-stv 1.0 1.0) (DEFAULT_AV)
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
; return the handle to ExecutionOutputLink, which would be used by 'connect_demand_goal' function
;
; The updater of the demand value is a combo script, named after demand_name with suffix "Updater"
;
; DemandSchema/DemandValue is represented as:
;
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "demand_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "demand_schema_name"
;             ListLink (empty)
;
; DemandValue is the output of DemandSchema.
;

(define (add_demand_schema demand_name default_value)
    (let ( (schema_handle (ExecutionOutputLink (stv 1.0 1.0) (DEFAULT_AV)
                              (GroundedSchemaNode (string-append (string-trim-both demand_name) "Updater") )
                              (ListLink)
                          );ExecutionOutputLink
           );schema_handle    
         )    

        (AtTimeLink (stv 1.0 1.0)
            (TimeNode CURRENT_TIMESTAMP)

            (SimilarityLink (cog-new-stv 1.0 1.0) (DEFAULT_AV)
                (NumberNode (number->string default_value) )
                schema_handle
            );SimilarityLink
        );AtTimeLink

        schema_handle

    );let
);define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
; 
; Connect a Demand and a Goal given demand_schema_handle, goal_handle, min_value and max_value
;
; The Goal here should be a Final Goal, also known as Demand Goal, 
; which is the starting point of the planer (backward chaining).
; 
; The connection is represented as:
;
; SimultaneousEquivalenceLink
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "demand_name_goal" 
;         ListLink (empty)
;     EvaluationLink
;         GroundedPredicateNode: "FuzzyWithin"
;         ListLink
;             NumberNode: "min_acceptable_value"
;             NumberNode: "max_acceptable_value"
;             ExecutionOutputLink
;                 GroundedSchemaNode: "demand_schema_name"
;                 ListLink (empty)
;

(define (connect_demand_goal demand_schema_handle goal_handle min_acceptable_value max_acceptable_value)
    (SimultaneousEquivalenceLink (cog-new-stv 1.0 1.0) (DEFAULT_AV)
        goal_handle

        (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)
            (GroundedPredicateNode "FuzzyWithin")  
            (ListLink 
                (NumberNode (number->string min_acceptable_value) )
                (NumberNode (number->string max_acceptable_value) )
                demand_schema_handle
            );ListLink
        );EvaluationLink

    );SimultaneousEquivalenceLink
);define

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
; A Goal is an EvaluationLink with a PredicateNode, which can be represented as below:
;
; EvaluationLink
;     PredicateNode "goal_pred_name"
;     ListLink
;         Node:arguments
;         ...
;
; There are two kinds of Goals, Final Goal and Intermediate Goal.
;
; A Final Goal, also known as Demand Goal, is to keep a specific Demand in a suitable range, 
; Which is the starting point of backward chaining. 
;
; While an Intermediate Goal should be used as other Rule's Precondition. 
;

(define (add_goal goal_pred_name . arguments)
    (EvaluationLink (DEFAULT_AV) 
        (PredicateNode (string-trim-both goal_pred_name) )

        (ListLink 
            (apply parse_arguments arguments)
        )
    );EvaluationLink
);define

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add GPN (GroundedPredicateNode) Precondition to AtomSpace.
;
; There are two kinds of Preconditions: 
; 
; 1. One is an EvaluationLink with a PredicateNode. 
;
; Since this kind of Precondition is used as other Rule's Goal, say Intermediate Goal, 
; there's no special function to create it, just use "add_goal" function below. 
;
; EvaluationLink
;     PredicateNode "precondition_name"
;     ListLink
;         Node:arguments
;         ...
;
; 2. Another is an EvaluationLink with a GroundedPredicateNode, say GPN Precondition,  
; which's Truth Value can only be evaluated by corresponding combo script.
;
; You should use this "add_gpn_precondition" function to create it. 
; schema_name should be exactly the same as the function name of the corresponding combo script
;
; EvaluationLink
;     GroundedPredicateNode "schema_name"
;     ListLink 
;         Node:arguments
;         ...
;

(define (add_gpn_precondition schema_name . arguments)
    (EvaluationLink (DEFAULT_STV) (DEFAULT_AV)

       (GroundedPredicateNode (string-trim-both schema_name) )

       (ListLink 
           (apply parse_arguments arguments)
       )

    );EvaluationLink
);define

; NULL_PRECONDITION is a dummy Goal that is always satisfied!
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
    (add_goal "NoPrecondition")
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Action to AtomSpace.
;
; An Action is an ExecutionLink with a GroundedSchemaNode, 
; which's Truth Value can only be evaluated by corresponding combo script. 
; 
; It can be represented as follows:
;
; ExecutionLink
;     GroundedSchemaNode "schema_name"
;     ListLink
;         Node:arguments
;         ...
;

(define (add_action schema_name . arguments)
    (ExecutionLink (DEFAULT_STV) (DEFAULT_AV)

        (GroundedSchemaNode (string-trim-both schema_name) )

        (ListLink 
            (apply parse_arguments arguments)
        )

    );ExecutionLink
);define

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
    (ExecutionLink (stv 1.0 1.0) (DEFAULT_AV)
        (GroundedSchemaNode "DoNothing")
        (ListLink)
    );ExecutionLink
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Add Rule (a PredictiveImplicationLink) to AtomSpace given Handles of Goal, Action and Preconditions
;
; A Rule is represented as below (AtTimeLink is missing currently): 
;
; PredictiveImplicationLink
;     AndLink
;         AndLink
;             EvaluationLink
;                 GroundedPredicateNode "precondition_1_name"
;                 ListLink
;                     Node:arguments
;                     ...
;             EvaluationLink
;                 PredicateNode         "precondition_2_name"
;                 ListLink
;                     Node:arguments
;                     ...
;             ...
;                        
;         ExecutionLink
;             GroundedSchemaNode "schema_name"
;             ListLink
;                 Node:arguments
;                 ...
;
;     EvaluationLink
;         (SimpleTruthValue indicates how well the demand is satisfied)
;         (ShortTermInportance indicates the urgency of the demand)
;         PredicateNode: "demand_name_goal" 
;         ListLink (empty)
;
; For each Rule, there's only a Goal, an Action and a bunch of Preconditions. 
; And all these Preconditions should be grouped in an AndLink.
; If you want to use OrLink, then just split the Rule into several Rules.
; For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.  
;
; 1. A Goal is an EvaluationLink with a PredicateNode, which can be represented as below:
;
; EvaluationLink
;     PredicateNode "goal_name"
;     ListLink
;         Node:arguments
;         ...
;
; There are two kinds of Goals, Final Goal and Intermediate Goal.
;
; A Final Goal, also known as Demand Goal, is to keep a specific Demand in a suitable range, 
; Which is the starting point of backward chaining. 
;
; While an Intermediate Goal should be used as other Rule's Precondition. 
;
; 2. An Action is an ExecutionLink with a GroundedSchemaNode, 
;    which's Truth Value can only be evaluated by corresponding combo script. 
;    It can be represented as follows:
;
; ExecutionLink
;     GroundedSchemaNode "schema_name"
;     ListLink
;         Node:arguments
;         ...
;
; 3. There are two kinds of Preconditions. 
; 
; One is an EvaluationLink with a PredicateNode, which may be set to other Rule's Goal, 
; say Intermediate Goal.
;
; EvaluationLink
;     PredicateNode "precondition_name"
;     ListLink
;         Node:arguments
;         ...
;
; Another is an EvaluationLink with a GroundedPredicateNode, say GPN Precondition, 
; which's Truth Value can only be evaluated by corresponding combo script, 
;
; EvaluationLink
;     GroundedPredicateNode "schema_name"
;     ListLink 
;         Node:arguments
;         ...
;

; TODO: Use PredictiveImplicationLink instead of ImplicationLink

(define (add_rule tv_handle goal_handle action_handle . precondition_handles)
    (ImplicationLink (DEFAULT_AV) tv_handle 
        (AndLink 
             (AndLink
                 precondition_handles
             )
            action_handle
        );AndLink

        goal_handle
    );PredictiveImplicationLink
);define


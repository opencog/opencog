;
; Helper functions used by all sorts of psi scheme scripts
;
; @author Jinhua Chua <JinhuaChua@gmail.com>
; @date   2011-11-25
;

; Initialize seed of pseudo-random generator using current time
(let ( (time (gettimeofday) )
     )
    (set! *random-state*
         (seed->random-state
             (+ (car time) (cdr time) )
         )
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Fuzzy logic related functions
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Helper functions of processing query AtomSpace via pattern matcher
;

; Return the scheme list of querying result given the ListLink containing it,
; this function will delete the ListLink once return
(define (unpack_query_result query_result_list_link)
    (let* ( (query_result_list (cog-outgoing-set query_result_list_link) )
          )

          (cog-delete query_result_list_link)

          query_result_list
    )
)

; Return the scheme list of querying result given BindLink (using cog-bind),
; return an empty list if fails
(define (query_atom_space bind_link)
    (let* ( (query_result_list_link (cog-bind bind_link) )
          )

          (unpack_query_result query_result_list_link)
    )
)

; Same as above. Return the scheme list of querying result given
; BindLink (using cog-bind-crisp),  return an empty list if fails
(define (query_atom_space_crisp bind_link)
    (let* ( (query_result_list_link (cog-bind bind_link) )
          )

          (unpack_query_result query_result_list_link)
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Helper functions about truth value
;

(define (get_truth_value_mean truth_value)
    (assoc-ref (cog-tv->alist truth_value) 'mean)
)

(define (get_truth_value_confidence truth_value)
    (assoc-ref (cog-tv->alist truth_value) 'confidence)
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Getter/ Setter of Modulators, Demands
;

; Modulator is represented as:
;
; AtTimeLink (stv 1. 1.0)
;     TimeNode "timestamp"
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "modulator_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "modulator_schema_name"
;
; Demand is represented as:
;
; AtTimeLink (stv 1.0 1.0)
;     TimeNode "timestamp"
;     SimilarityLink (stv 1.0 1.0)
;         NumberNode: "demand_value"
;         ExecutionOutputLink (stv 1.0 1.0)
;             GroundedSchemaNode: "demand_schema_name"

; BindLink used by cog-bind to search AtTimeLink of a modulator or a demand,
; given the corresponding updater (ExecutionOutputLink)
(define (find_modulator_or_demand_at_time_link updater_execution_output_link)
    (if (cog-atom? updater_execution_output_link)
        (BindLink
            ; Variables to be used
            (ListLink
                (TypedVariableLink
                    (VariableNode "$var_time_node_type")
                    (TypeNode "TimeNode")
                )

                (TypedVariableLink
                    (VariableNode "$var_number_node_type")
                    (TypeNode "NumberNode")
                )

            ); ListLink

            ; Pattern to be searched
            (AtTimeLink
                (VariableNode "$var_time_node_type")
                (SimilarityLink
                    (VariableNode "$var_number_node_type")
                    updater_execution_output_link
                )
            )

            ; Return values encapsulated by a ListLink
            (ListLink
                (VariableNode "$var_time_node_type")
                (VariableNode "$var_number_node_type")
                updater_execution_output_link
            )

        ); BindLink

        (list)
    ); if
); define

; Return the ExecutionOutputLink of the updater given modulator or demand name,
; if fails return an empty list
;
(define (get_modulator_or_demand_updater modulator_or_demand_name)
    (ExecutionOutputLink
        (GroundedSchemaNode
            (string-append (string-trim-both modulator_or_demand_name) "Updater")
        )
    )
)

; Return a scheme list containing the information of a modulator or demand
; at all the time points given modulator or demand name, the return result
; would be something like
;
; ( ( (TimeNode "timestamp1") (NumberNode "value1") (ExecutionOutputLink "updater1") )
;   ( (TimeNode "timestamp2") (NumberNode "value2") (ExecutionOutputLink "updater2") )
;   ( (TimeNode "timestamp3") (NumberNode "value3") (ExecutionOutputLink "updater3") )
;   ...
; )
;
; @note the modulator name should have a 'Modulator' suffix, while the demand name
;       should end with 'Demand'
;       the updater name should end with 'Updater'
;
(define (get_modulator_or_demand modulator_or_demand_name)
    (let* ( (updater_execution_output_link
                (get_modulator_or_demand_updater modulator_or_demand_name)
            )

            (find_modulator_or_demand (list) )
            (modulator_or_demand_list (list) )
          )

          (if (null? updater_execution_output_link)
              (print_debug_info INFO_TYPE_FAIL "get_latest_modulator_or_demand"
                                (string-append "Failed to retrieve "
                                                modulator_or_demand_name
                                                " from AtomSpace."
                                )
              )

              (begin
                  (set! find_modulator_or_demand
                      (find_modulator_or_demand_at_time_link updater_execution_output_link)
                  )

                  (set! modulator_or_demand_list
                      (query_atom_space find_modulator_or_demand)
                  )

                  (set! modulator_or_demand_list
                      (map-in-order unpack_query_result modulator_or_demand_list)
                  )
              ); begin
          ); if

          ; Return the query result
          modulator_or_demand_list
    ); let*
)

; Return the latest modulator or demand information given modulator or demand name.
; The return result would be something like
;
; ( (TimeNode "timestamp")
;   (NumberNode "value")
;   (ExecutionOutputLink "updater")
; )
;
(define (get_latest_modulator_or_demand modulator_or_demand_name)
     (let* ( (modulator_or_demand_list (get_modulator_or_demand modulator_or_demand_name) )
             (latest_modulator_or_demand (list) )
             (latest_timestamp (list) )
           )

           ; Pick up the modualtor or demand with the latest (largest) timestamp
           (map-in-order
               (lambda (modulator_or_demand)
                   (let* ( (time_node (car modulator_or_demand) )
                           (timestamp (string->number (cog-name time_node) ) )
                         )

                         (if (or (null? latest_timestamp)
                                 (> timestamp latest_timestamp)
                             )

                             (begin
                                 (set! latest_modulator_or_demand modulator_or_demand)
                                 (set! latest_timestamp timestamp)
                             )

                         ); if
                   ); let*
               ); lambda

               modulator_or_demand_list

           ); map-in-order

           ; Return the information of the latest modulator or demand
           latest_modulator_or_demand
     ); let*
)

; Return the latest modulator or demand value given modulator or demand name.
; If fails to retrieve the modulator or demand from AtomSpace, it would return a random
; value in [0, 1]
(define (get_latest_modulator_or_demand_value modulator_or_demand_name)
    (get_latest_predicate_truth_value_mean modulator_or_demand_name)
)

; Save the modulator or demand value to AtomSpace given modulator or demand name,
; the udpated value and the timestamp.
; Return the newly created AtTimeLink or an empty list once fails
(define (set_modulator_or_demand_value modulator_or_demand_name updated_value timestamp)
    (let* ( (updater_execution_output_link
                (get_modulator_or_demand_updater modulator_or_demand_name)
            )

            (at_time_link (list) )
          )

          (if (null? updater_execution_output_link)
              (print_debug_info INFO_TYPE_WARN "set_modulator_or_demand_value"
                                (string-append "Failed to set value for "
                                                modulator_or_demand_name
                                                ". Because we can not retrieve it from AtomSpace."
                                )
              )

              (set! at_time_link
                  (AtTimeLink (stv 1.0 1.0)
                      (TimeNode (number->string timestamp) )
                      (SimilarityLink
                          (NumberNode (number->string updated_value) )
                          updater_execution_output_link
                      )
                  ); AtTimeLink
              )
          ); if

          ; Return the newly created AtTimeLink
          at_time_link
    ); let*
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Getter/ Setters for EvaluationLink, which are useful for Demand Goals, Feelings etc.
;

; BindLink used by cog-bind to search the EvaluationLink given PredicateNode name
;
; The format of EvaluationLink is as follows:
;
; EvaluationLink
;     PredicateNode  "predicate_name"
;     ListLink
;         ...
;

(define (find_evaluation_link predicate_name)
    (BindLink
        ; Variables to be used
        (ListLink
            (TypedVariableLink
                (VariableNode "$var_list_link_type")
                (TypeNode "ListLink")
            )
        )

        ; Pattern to be searched
        (EvaluationLink
            (PredicateNode
                (string-trim-both predicate_name)
            )
            (VariableNode "$var_list_link_type")
        )

        ; Return values
        (EvaluationLink
            (PredicateNode
                (string-trim-both predicate_name)
            )
            (VariableNode "$var_list_link_type")
        )

    ); BindLink
)

; Return the single EvaluationLink given the predicate_name
; if failed or found multiple EvaluationLink, return an empty list
(define (get_evaluation_link predicate_name)
    (let* ( (evaluation_link_list
                (query_atom_space (find_evaluation_link predicate_name) )
            )
          )

          (if (null? evaluation_link_list)
              ; If failed to find the EvaluationLink, return an empty list
              (list)

              (if (equal? (length evaluation_link_list) 1)
                  ; If found the single EvaluationLink, return it
                  (car evaluation_link_list)

                  ; If found multiple EvaluationLink, return an empty list
                  (begin
                      (print_debug_info INFO_TYPE_WARN
                          "get_evaluation_link"
                          (string-append "Number of EvaluationLink containing "
                                         "PredicateNode: " predicate_name
                                         " should be exactly 1. But got "
                                         (number->string (length evaluation_link_list) )
                          )
                      )
                      (list)
                  )
              ); if
          ); if

    ); let*
); define

; Return the truth value of EvaluationLink given the predicate name.
; If fails to retrieve the EvaluationLink from AtomSpace, it would return a
; random SimpleTruthValue with both mean and confidence in [0, 1]
(define (get_predicate_truth_value predicate_name)
    (let* ( (evaluation_link (get_evaluation_link predicate_name) )
          )

          (if (null? evaluation_link)
              (begin
                  (print_debug_info INFO_TYPE_WARN "get_predicate_truth_value"
                                    (string-append "Failed to retrieve EvaluationLink "
                                                    "containing PredicateNode: "
                                                    predicate_name " from AtomSpace. "
                                                    "Return a random SimpleTruthValue "
                                                    "in [0, 1] instead."
                                    )
                  )

                  (stv (random:uniform) (random:uniform) )
              ); begin

              (cog-tv evaluation_link)
          ); if

    ); let*
); define

; Return the mean of the truth value of EvaluationLink given the
; PredicateNode name.
; If fails to retrieve the corresponding EvaluationLink, return a random number
; in [0, 1]
(define (get_predicate_truth_value_mean predicate_name)
    (get_truth_value_mean (get_predicate_truth_value predicate_name) )
)

; Set the truth value of EvaluationLink given predicate name and return the
; EvaluationLink with udpated truth value.
; If fails, return an empty scheme list.
(define (set_predicate_truth_value predicate_name truth_value)
    (let* ( (evaluation_link (get_evaluation_link predicate_name) )
          )

          (if (null? evaluation_link)
              (print_debug_info INFO_TYPE_WARN "set_predicate_truth_value"
                                (string-append "Failed to set truth value for "
                                                "EvaluationLink containing "
                                                "PredicateNode: " predicate_name
                                                " from AtomSpace. "
                                                "Because we can not retrieve it "
                                                "from AtomSpace."
                                )
              )

              (cog-set-tv! evaluation_link truth_value)
          ); if

          ; Return the EvaluationLink with updated truth value
          ; or an empty scheme list if fails.
          evaluation_link
    ); let*
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Getter/ Setters for LatestLink containing EvaluationLink, which are useful for
; all kinds of physiological levels, such as energy, hungry, thirst etc.
;

; BindLink used by cog-bind to search the latest AtTimeLink given PredicateNode name
;
; The format of EvaluationLink is as follows:
;
; AtTimeLink (physiological level is stored as truth value here)
;     TimeNode "timestamp"
;     EvaluationLink
;         PredicateNode  "predicate_name"
;         ListLink
;         ...
;

(define (find_latest_at_time_link predicate_name)
    (BindLink
        ; Variables to be used
        (ListLink
            (TypedVariableLink
                (VariableNode "$var_time_node_type")
                (TypeNode "TimeNode")
            )
            (TypedVariableLink
                (VariableNode "$var_list_link_type")
                (TypeNode "ListLink")
            )
        )

        ; Pattern to be searched
        (LatestLink
            (AtTimeLink
                (VariableNode "$var_time_node_type")
                (EvaluationLink
                    (PredicateNode
                        (string-trim-both predicate_name)
                    )
                    (VariableNode "$var_list_link_type")
                )
            )
        ); LatestLink

        ; Return values
        (AtTimeLink
            (VariableNode "$var_time_node_type")
            (EvaluationLink
                (PredicateNode
                    (string-trim-both predicate_name)
                )
                (VariableNode "$var_list_link_type")
            )
        ); AtTimeLink

    ); BindLink
)

; Return the latest AtTimeLink given the predicate_name
; if failed or found multiple EvaluationLink, return an empty list
(define (get_latest_at_time_link predicate_name)
    (let* ( (latest_at_time_link_list
                (query_atom_space (find_latest_at_time_link predicate_name) )
            )
          )

          (if (null? latest_at_time_link_list)
              ; If failed to find the latest AtTimeLink, return an empty list
              (list)

              (if (equal? (length latest_at_time_link_list) 1)
                  ; If found the latest AtTimeLink, return it
                  (car latest_at_time_link_list)

                  ; If found multiple latest AtTimeLink, return an empty list
                  (begin
                      (print_debug_info INFO_TYPE_WARN
                          "get_latest_at_time_link"
                          (string-append "Number of latest AtTimeLink containing "
                                         "PredicateNode: " predicate_name
                                         " should be exactly 1. But got "
                                         (number->string (length latest_at_time_link_list) )
                          )
                      )
                      (list)
                  )
              ); if
          ); if

    ); let*
); define

; Return the truth value of latest AtTimeLink given the predicate name.
; If fails to retrieve the latest AtTimeLink from AtomSpace, it would return a
; random SimpleTruthValue with both mean and confidence in [0, 1]
(define (get_latest_predicate_truth_value predicate_name)
    (let* ( (latest_at_time_link (get_latest_at_time_link predicate_name) )
          )

          (if (null? latest_at_time_link)
              (begin
                  (print_debug_info INFO_TYPE_WARN "get_latest_predicate_truth_value"
                                    (string-append "Failed to retrieve latest AtTimeLink "
                                                    "containing PredicateNode: "
                                                    predicate_name " from AtomSpace. "
                                                    "Return a random SimpleTruthValue "
                                                    "in [0, 1] instead."
                                    )
                  )

                  (stv (random:uniform) (random:uniform) )
              ); begin

              (cog-tv latest_at_time_link)
          ); if

    ); let*
); define

; Return the mean of the truth value of latest AtTimeLink given the
; PredicateNode name.
; If fails to retrieve the latest AtTimeLink, return a random number
; in [0, 1]
(define (get_latest_predicate_truth_value_mean predicate_name)
    (get_truth_value_mean (get_latest_predicate_truth_value predicate_name) )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

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

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
;
; Get handle to the owner/ self
;
; The ownership (i.e. OWNERSHIP_PREDICATE_NAME) stored in AtomSpace is as follows:
;
; (EvaluationLink (stv 1 0) (av -9 1 0)
;     (PredicateNode "owns" (av -9 1 0))
;     (ListLink (av 0 1 0)
;         (AvatarNode "id_265" (av 0 1 0))
;         (PetNode "id_73565" (av 0 1 0))
;     )
; )
;

(define (get_owner)
    (let ( (ownership_evaluation_link_list (get_evaluation_link "owns") )
           (list_link (list) )
         )

         (if (null? ownership_evaluation_link_list)
             (list)

             (begin
                 (set! list_link
                     (list-ref (cog-outgoing-set ownership_evaluation_link_list) 1)
                 )

                 (list-ref (cog-outgoing-set list_link) 0)
             )
         ); if

    ); let
)

(define (get_self)
    (let ( (ownership_evaluation_link_list (get_evaluation_link "owns") )
           (list_link (list) )
         )

         (if (null? ownership_evaluation_link_list)
             (list)

             (begin
                 (set! list_link
                     (list-ref (cog-outgoing-set ownership_evaluation_link_list) 1)
                 )

                 (list-ref (cog-outgoing-set list_link) 1)
             )
         ); if

    ); let

)


; Get proximity
;
; (EvaluationLink (stv 0.65710086 0.0012484394) (av -9 1 0)
;     (PredicateNode "proximity" (av -9 1 0))
;     (ListLink
;         (AccessoryNode "id_73548" (av -9 0 0))
;         (AvatarNode "id_265" (av 0 1 0))
;    )
; )
;
; The truth value of 'proximity' is defined as
; SimpleTruthValue tv(1.0 - (distance/mapDiagonal), 1);
; See also ./opencog/embodiment/Control/PredicateUpdaters/SpatialPredicateUpdater.cc

(define (get_proximity object_a object_b)
    (let ( (proximity_evaluation_link (list) )
         )

         (set! proximity_evaluation_link
             (EvaluationLink
                 (PredicateNode "proximity")
                 (ListLink
                     object_a
                     object_b
                 )
             )
         ); set!

         (get_truth_value_mean (cog-tv proximity_evaluation_link) )
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;

; BindLink used by cog-bind to get all the AtTimeLink given PredicateNode name
;
; The format of EvaluationLink is as follows:
;
; AtTimeLink (physiological level is stored as truth value here)
;     TimeNode "timestamp"
;     EvaluationLink
;         PredicateNode  "predicate_name"
;         ListLink
;             ...

(define (find_at_time_link predicate_name)
    (BindLink
        ; Variables to be used
        (ListLink
            (TypedVariableLink
                (VariableNode "$var_time_node_type")
                (TypeNode "TimeNode")
            )
            (TypedVariableLink
                (VariableNode "$var_list_link_type")
                (TypeNode "ListLink")
            )
        )

        ; Pattern to be searched
        (AtTimeLink
            (VariableNode "$var_time_node_type")
            (EvaluationLink
                (PredicateNode
                    (string-trim-both predicate_name)
                )
                (VariableNode "$var_list_link_type")
            )
        )

        ; Return values
        (AtTimeLink
            (VariableNode "$var_time_node_type")
            (EvaluationLink
                (PredicateNode
                    (string-trim-both predicate_name)
                )
                (VariableNode "$var_list_link_type")
            )
        ); AtTimeLink

    ); BindLink
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Helper functions related to ReferenceLink
;
; BindLink used by pattern matcher to get the ReferenceLink given the first
; outgoing
(define (find_reference_link first_outgoing)
    (BindLink
        ; Variables
        (VariableNode "$var_any")

        ; Pattern to be searched
        (ReferenceLink
            first_outgoing
            (VariableNode "$var_any")
        )

        ; Return result
        (ReferenceLink
            first_outgoing
            (VariableNode "$var_any")
        )

    ); BindLink
)

; Return the ReferenceLink given the first outgoing
(define (get_reference_link first_outgoing)
    (let ( (reference_link_list
                (query_atom_space (find_reference_link first_outgoing) )
           )
         )

         (if (null? reference_link_list)
             (list)
             (car reference_link_list)
         )
    )
)

; Return the reference of the given atom
(define (get_reference first_outgoing)
    (let ( (reference_link (get_reference_link first_outgoing) )
         )

         (if (null? reference_link)
             (list)
             (list-ref (cog-outgoing-set reference_link) 1)
         )
    )
)

; Remove the old ReferenceLink (if any) holding the first_outgoing and
; create a new ReferenceLink containing the given outgoings,
; return the newly created ReferenceLink
(define (update_reference_link first_outgoing second_outgoing)
    (let ( (old_reference_link (get_reference_link first_outgoing) )
         )

         (if (not (null? old_reference_link) )
             (cog-delete old_reference_link)
         )

         (ReferenceLink
             first_outgoing
             second_outgoing
         )
    )
)

;||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||
;
; Miscellaneous
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

; Return #t if given atom is a Link
(define (cog-link? atom)
    (cog-subtype? 'Link (cog-type atom) )
)

; Return #t if given atom is an Node
(define (cog-node? atom)
    (cog-subtype? 'Node (cog-type atom) )
)

; Return a string representing the given atom
(define (atom_as_string atom)
    (cond
       ( (not (cog-atom? atom) )
         ""
       )

       ( (cog-node? atom)
         (string-append (cog-name atom) " ")
       )

       (else
         (string-append
           "[" (fold-right string-append "" (map-in-order atom_as_string (cog-outgoing-set atom) ) )
               "<" (number->string (get_truth_value_mean (cog-tv atom) ) ) " "
                   (number->string (get_truth_value_confidence (cog-tv atom) ) ) ">"
           "]"
         )
       )
    ); cond
)

(define (get_changes_with_tv)
    (let* ( (changes_with_tv (cog-incoming-set (PredicateNode "change_with_tv")) )
          )

          (set! changes_with_tv
              (filter
                   (lambda (change_candidate)
                       (eqv? (cog-type change_candidate) 'ReferenceLink)
                   )
                   changes_with_tv
              )
          )

          changes_with_tv
    )
)

(define (get_changes_with_arg)
    (let ( (list_link (get_reference (PredicateNode "change_with_arg")) )
         )
         (if (null? list_link)
             (list)
             (cog-outgoing-set list_link)
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

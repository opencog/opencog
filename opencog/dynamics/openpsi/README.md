https://en.wikipedia.org/wiki/Belief%E2%80%93desire%E2%80%93intention_software_model#BDI_agents has some similarities.
# OpenPsi

Work is progressing on this, so is not fully functional. The main task that is
being performed on it is refactoring, so as to make it run independent of the
embodiment code. When the refactoring is finalized it would be possible to use
OpenPsi to drive the goal driven dynamics of mind-agents/process, embodiment
agents as well as other opencog modules/components.

### Resources
* http://wiki.opencog.org/w/OpenPsi_%28Embodiment%29
* http://wiki.hansonrobotics.com/w/Emotion_modeling
* [MicroPsi publications](http://micropsi.com/publications/publications.html)
* [MicroPsi source code]()
* [Principles of Synthetic Intelligence](http://wiki.humanobs.org/_media/public:events:agi-summerschool-2012:psi-oup-version-draft-jan-08.pdf)

Then the cognitive schematic
     Contex & Procedure ==> Goal

### Usage
After starting the cogserver
1. loadmodule opencog/dynamics/openpsi/libOpenPsi.so

2. Start any of the agents
    * agents-start opencog::PsiActionSelectionAgent
    * agents-start opencog::PsiModulatorUpdaterAgent
    * agents-start opencog::PsiDemandUpdaterAgent

3. Stop any of the started agents
    * agents-stop opencog::PsiActionSelectionAgent
    * agents-stop opencog::PsiModulatorUpdaterAgent
    * agents-stop opencog::PsiDemandUpdaterAgent

OpenPsi Rules aren't defined yet so nothing interesting occurs.

### How are OpenPsi components represented in AtomSpace?
1. Modulator is represented as:
```
    (AtTimeLink (stv 1.0 1.0)
        (TimeNode "timestamp")
        (SimilarityLink
            (NumberNode "modulator_value")
            (ExecutionOutputLink (stv 1.0 1.0)
                (GroundedSchemaNode "modulator_nameUpdater")
            )
        )
    )
```
To add a modulator use `(add_modulator modulator_name default_value)` where
'default_value' is a number in [0, 1]. How to choose the default_value ????

2. DemandSchema=Demand is represented as:
```
    (AtTimeLink (stv 1.0 1.0)
        (TimeNode "timestamp")
        (SimilarityLink
            (NumberNode: "demand_value")
            (ExecutionOutputLink (stv 1.0 1.0)
                (GroundedSchemaNode: "demand_schemaUpdater")
            )
        )
    )

```

To add a demand use ``(add_demand_schema demand_name default_value)`. The demand_value is updated by the function 'demand_schemaUpdater' and it is the
metric of the demand-goal. Depending on the demand_value there could be various
goals. A demand without a goal is meaningless

3. DemandGoal=Goal
Goals are associated with each demand. Each none grounded goal or precondition
or context should have a corresponding GroundedPredicateNode to check if the
goal or precondition has been achieved or not. They are related via an SimultaneousEquivalenceLink as follows:

context are of two types
1. The existance of a state
2. The absence of a state. For eg. a goal hasn't been achieved.
where a state is defined by the a

SimultaneousEquivalenceLink
 EvaluationLink
     PredicateNode "none_grounded_goal_or_precondition_name"
     ListLink
         ...

 EvaluationLink
     GroundedPredicateNode "updater_schema_name"
     ListLink
         ...

A special case is the DemandGoal, which uses a "fuzzy_within" scheme function
to calculate its truth value. While the XxxDemandUpdater is a scheme function
of updating the demand level (not the truth value of it). what ??

Take CertaintyDemand as an example, the CertaintyDemandUpdater is responsible
for updating the certainty level of the agent via a bunch of information stored
in AtomSpace, while the truth value of the CertaintyDemandGoal is calculated
via fuzzy_within function, which would evaluate how well the certainty level is
within the suitable range [min_acceptable_value, max_acceptable_value].

```
    (SimultaneousEquivalenceLink
        (EvaluationLink
            (PredicateNode "XxxDemandGoal")
        )
        (EvaluationLink
            (GroundedPredicateNode "fuzzy_within")
            (ListLink
                (NumberNode "min_acceptable_value")
                (NumberNode "max_acceptable_value")
                (ExecutionOutputLink
                    (GroundedSchemaNode "XxxDemandUpdater")
                )
            )
        )
    )
```

||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

 Finally, a Psi Rule is represented as:

 ImplicationLink (truth value indicates the probability to be selected while planning)
     AndLink
         AndLink
             precondition_1 (truth value indicates how well the precondition or subgoal is satisfied)
             subgoal_1
             ...

         ExecutionLink (truth value indicates whether the action has been done successfully)
             action

     EvaluationLink (truth value indicates how well the goal is satisfied)
         goal


Add Rule (an ImplicationLink) to AtomSpace given Handles of Goal, Action and Preconditions

For each Rule, there's only a Goal, an Action and a bunch of Preconditions.
And all these Preconditions should be grouped in an AndLink.
If you want to use OrLink, then just split the Rule into several Rules.
For the efficiency and simplicity of the planer (backward chainging), NotLink is forbidden currently.

1. Psi Rule is represented as follows:

ImplicationLink (higher truth value means higher probability of selection when planning)
 AndLink
     AndLink
         Preconditions
     Action
 Goal

2. Goal is represented as:

EvaluationLink
 PredicateNode "goal_name_1"
 ListLink
     ...

or

EvaluationLink
 GroundedPredicateNode "gpn_goal_name_2"
 ListLink
     ...

3. Preconditions are represented as:

AndLink
 EvaluationLink
     PredicateNode "sub_goal_name_1"
     ListLink
         ...

 EvaluationLink
     GroundedPredicateNode "gpn_sub_goal_name_2"
     ListLink
         ...

4. Action is represented as:

ExecutionLink (truth value means if the action has been done successfully)
 GroundedSchemaNode "schema_name_1"
 ListLink
     ...

Note: For each goal or precondition, we can use PredicateNode or GroundedPredicateNode
   within EvaluationLink. The truth value means how well the goal or
   precondition are satisfied. The attention value means the urgency.

   Each none grounded goal or precondition should have a corresponding
   GroundedPredicateNode to check if the goal or precondition has been
   achieved or not. They are related via an SimultaneousEquivalenceLink
   as follows:

   SimultaneousEquivalenceLink
       EvaluationLink
           PredicateNode "none_grounded_goal_or_precondition_name"
           ListLink
               ...
       EvaluationLink
           GroundedPredicateNode "updater_schema_name"
           ListLink
               ...


||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||||

 Reference:

     http://wiki.opencog.org/w/OpenCogPrime:FunctionNotation
     http://wiki.opencog.org/w/TimeServer

******************************************************************************


### Algorithm
1. What it does?
2. Why it does it that way?

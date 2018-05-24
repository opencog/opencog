# Procedures

Here is a list of procedures (predicates and schemas) that can be used in GHOST. Predicates should be used in the context of a rule while schemas should be used in the action.

## Predicates

```
^person_appears(#:optional face-id)
```
[perception_ctrl.py::46](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L46)->[procedures.scm::135](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/procedures.scm#L135)->[predicates.scm::93](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L93)

This predicate triggers when a face appears in the visual perception system. Optionally a face-id can be provided.

-----

```
^person_smiles(#:optional face-id)
^person_angry(#:optional face-id)
...?
```
[perception_ctrl.py::58](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L58)->[procedures.scm::149](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/procedures.scm#L149)->[predicates.scm::100](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L100)

-----

```
^person_eyes_closed(#:optional face-id)
```
[perception_ctrl.py::70](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L70)->[procedures.scm::225](https://github.com/opencog/opencog/blob/34ee04d02cec967f7afa8999ce63879dec4c65f5/opencog/ghost/procedures/procedures.scm#L225)->[🔥predicates.scm](#)

  - Currently being implemented.

-----

```
^person_talking(#:optional face-id)
^person_not_talking(#:optional face-id)
```
[perception_ctrl.py::81](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L81)->[procedures.scm::181](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/procedures.scm#L181)->[predicates.scm::114](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L114)


-----

```
^word_perceived(word #:optional time-interval)
```
[perception_ctrl.py::92](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L92)->[procedures.scm::163](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/procedures.scm#L163)->[predicates.scm::125](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L125)


-----

```
^after_min(minutes #:optional timer-id)
```
[predicates.scm::133](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L133)

  - 🔥 This needs testing and example.

-----

```
^after_user_started_talking(seconds)
^after_user_stopped_talking(seconds)

```
[perception_ctrl.py::81](https://github.com/opencog/ghost_bridge/blob/1e554bbf79ea890f8fdc1b0ce6681f65e6a38869/src/ghost_bridge/perception_ctrl.py#L81)->[procedures.scm::181](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/procedures.scm#L181)->[predicates.scm::151](https://github.com/opencog/opencog/blob/28a1c3aa6074616e141d26b8c51a324203236ace/opencog/ghost/procedures/predicates.scm#L151)

  - 🔥 This needs testing, once the signal of face-talking is clearer.



## Schemas

```
- animation
- expression
```
These action predicates need to be updated in accordance to GHOST bridge. Proposed new schemas are:
```
^emotionAnimation(type, strength, duration)
^emotionStatic(type, strength)
^gesture(animation, strength, duration)
```
There might be additional predicates for blink, soma, etc.

-----

```
^decrease_urge(goalID, amount)
^increase_urge(goalID, amount)
```
Decreases and increases the urge level of any given goal. Tested and works.

-----

```
^start_timer(#:optional timer-id)
```
Starts a timer. Not sure if this needs to be called before using timer.

## Deprecated // Debugging // Do not Use

```
^stimulate_words(words)
^stimulate_concepts(concepts)
^stimulate_rules(rule-labels)
```
Raw functions for stimulating WordNodes, ConceptNodes, Rule by labels.

-----

```
^set_rule_sti(rule-label val)
```
Specifically set STI of a rule by label.

-----

```
- max_sti_words
- max_sti_concepts
- max_sti_rules
```
Maximize STI of words, concepts, rules.

-----

```
- min_sti_words
- min_sti_concepts
- min_sti_rules
```
Minimize STI of words, concepts, rules.

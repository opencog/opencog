# Progress summary of this week
* Beautiful version is available in my wiki (http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending)

## Now
### In Progress
* Find the method to get my link's truthvalue (link.tv is not match with my input).
* Implement blender selector by passing instance which is subclass of abstract class.
* Check for made blended node and trying to test with several case.
* Select nodes to blending.
  * Implement checking for some values in HebbianLink and SimilarityLink when 
 select node
  * Make the waiting queue of ConceptNode
  
### To do
**Think**

* (empty)

**Study**

* (empty)

**CodingCodingCoding**

* Decide whether or not to execute blending and prepare
  * Implement skipping blend and run the next cycle
  * Implement selecting ConceptNode which has best value (proper value)

### Pause
* Search for good algorithms to use in each step.
* Design about public API of my project.
  * I think I need to start creating mindagent as soon as possible, instead of 
 various considering about architecture.


## Result
### Done
* Select nodes to blending.
  * Select by random
* Decide whether or not to execute blending and prepare.
  * Try blending immediately
* Make the simple, newly blended node and put in AtomSpace.
* Make "agent-step" command simulating.
* Make the links in newly blended node and correct some attribute value.

### Fail
* (empty)

### Cancel
* (empty)

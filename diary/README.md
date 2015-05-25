# Progress summary of this week
* Beautiful version is available in my wiki (http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending)

## Now
### In Progress
* Select nodes to blending.
  * Implement checking for some values in HebbianLink and SimilarityLink when 
 select node
  * Make the waiting queue of ConceptNode
* Decide whether or not to execute blending and prepare
  * Implement skipping blend and run the next cycle
  * Implement selecting ConceptNode which has best value (proper value)
### To do
**Think**

* (empty)

**Study**

* (empty)

**CodingCodingCoding**

* Check the conflict links in each node and remove.
  * Implement removing conflict links
* Detect and improve conflict links in newly blended node
  * Select one by random in 2^k possible things

### Pause
* Search for good algorithms to use in each step.
* Design about public API of my project.
  * I think I need to start creating mindagent as soon as possible, instead of 
 various considering about architecture.
* Implement correct truthvalue of link method selector by passing instance 
 which is subclass of abstract class.
* Make API document.

## Result
### Done
* Implement blender selector by passing instance which is subclass of abstract class.
  * Implement blender selector.
  * Implement test case selector.
* Check for made blended node and trying to test with several case.
  * Done. (Paul & Sally example, Debate with Kant example)
  * However, it doesn't make any sense at current time.

### Fail & Deadline Missed
* (empty)

### Cancel
* (empty)

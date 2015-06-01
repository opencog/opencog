# Progress summary of this week
* Beautiful version is available in my wiki
 (http://wiki.dong-min.kim/GSoC_2015_-_Conceptual_Blending)

## Now
### In Progress
* Try to apply STI in my test cases. (Currently test cases use 'BlendTarget' 
 which is only useful in development stage)
 
### To do
**Think**
* (empty)

**Study**
* Try to use Unified Rule Engine, PLN in python.

**CodingCodingCoding**
* Check the conflict links in each node and remove.
  * Implement removing conflict links
* Detect and improve conflict links in newly blended node
  * Select one by random in 2^k possible things
* Select nodes to blending.
  * Implement checking for some values in HebbianLink and SimilarityLink when 
 select node
    * Try to apply Attention Allocation agents
  * Make the waiting queue of ConceptNode
* Decide whether or not to execute blending and prepare
  * Implement skipping blend and run the next cycle
  * Implement selecting ConceptNode which has best value (proper value)
* Optimize implemented code and make simple documentation.

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
* Try to connect python debugger to cogserver.
:* Done.
:* Python's GIL(GlobalInterpreterLock) policy has little bug in Python 2.x, so
 debugging with PyCharm libraries should be use with some hack codes..

### Fail & Deadline Missed
* (empty)

### Cancel
* (empty)
